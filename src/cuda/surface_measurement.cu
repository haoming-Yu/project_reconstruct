// Measures the surface, i.e. computes vertex and normal maps from a depth frame
// This is CUDA code; compile with nvcc
// Author: Christian Diller, git@christian-diller.de

#include "include/common.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/ximgproc/edge_filter.hpp>

using cv::cuda::GpuMat;

namespace kinectfusion {
    namespace internal {
        namespace cuda {

            __global__
            void kernel_compute_vertex_map(const PtrStepSz<unsigned char> gpu_mask, 
                                           const PtrStepSz<float> depth_map, PtrStep<float3> vertex_map,
                                           const float depth_cutoff, const CameraParameters cam_params)
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;
                if (x >= depth_map.cols || y >= depth_map.rows)
                    return;

                unsigned char mask_value = gpu_mask.ptr(y)[x];
                float depth_value = depth_map.ptr(y)[x];
                if (mask_value == 0 || depth_value > depth_cutoff || depth_value <= 10.0f) {
                    depth_value = 0.f;
                } // outlier filter out
                Vec3fda vertex(
                        (x - cam_params.principal_x) * depth_value / cam_params.focal_x,
                        (y - cam_params.principal_y) * depth_value / cam_params.focal_y,
                        depth_value);
                vertex_map.ptr(y)[x] = make_float3(vertex.x(), vertex.y(), vertex.z());
            }

            __global__
            void kernel_compute_vertex_map(const PtrStepSz<float> depth_map, PtrStep<float3> vertex_map,
                                           const float depth_cutoff, const CameraParameters cam_params)
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                if (x >= depth_map.cols || y >= depth_map.rows)
                    return;

                float depth_value = depth_map.ptr(y)[x];
                if (depth_value > depth_cutoff) depth_value = 0.f; // Depth cutoff

                Vec3fda vertex(
                        (x - cam_params.principal_x) * depth_value / cam_params.focal_x,
                        (y - cam_params.principal_y) * depth_value / cam_params.focal_y,
                        depth_value);

                vertex_map.ptr(y)[x] = make_float3(vertex.x(), vertex.y(), vertex.z());
            }

            __global__
            void kernel_compute_normal_map(const PtrStepSz<float3> vertex_map, PtrStep<float3> normal_map, const CameraParameters cam_params)
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                // disable all the corner cases introduced by gpu allocation.
                if (x < 1 || x >= vertex_map.cols - 1 || y < 1 || y >= vertex_map.rows - 1)
                    return;

                Vec3fda ray_dir(
                    (x - cam_params.principal_x) / cam_params.focal_x, // x component
                    (y - cam_params.principal_y) / cam_params.focal_y, // y component
                    1.0f); // z component
                ray_dir.normalize();
                
                const Vec3fda left(&vertex_map.ptr(y)[x - 1].x);
                const Vec3fda right(&vertex_map.ptr(y)[x + 1].x);
                const Vec3fda upper(&vertex_map.ptr(y - 1)[x].x);
                const Vec3fda lower(&vertex_map.ptr(y + 1)[x].x);

                Vec3fda normal;

                if (left.z() == 0 || right.z() == 0 || upper.z() == 0 || lower.z() == 0)
                    normal = Vec3fda(0.f, 0.f, 0.f);
                else {
                    Vec3fda hor(left.x() - right.x(), left.y() - right.y(), left.z() - right.z());
                    Vec3fda ver(upper.x() - lower.x(), upper.y() - lower.y(), upper.z() - lower.z());

                    normal = hor.cross(ver);
                    normal.normalize();

                    // switch normal to point to the outer side of the mesh, with negative z value
                    // here normal vector has already been normalized. -> length == 1
                    if (normal.z() > 0)
                        normal *= -1; 
                        
                    ray_dir *= -1; // let direction of ray points back to the camera.
                    float dot_product = ray_dir.x() * normal.x() + ray_dir.y() * normal.y() + ray_dir.z() * normal.z(); // these two value are all normalized. thus dot_product == cos_value
                    // the angle between inversed ray_dir and normal value's range is [0, 180) on a spherical surface
                    // thus using cosine function makes sense. and for angle less than 60 degree is regarded to be useful.
                    // which means, the cosine value between [1, 1/2] is useful. -> the dot product >= 1/2 is useful for reconstruction
                    if (dot_product < 0.80f) { // 0.8660254037844386 -> sqrt(3) / 2
                        normal = Vec3fda(0.f, 0.f, 0.f);
                    }
                }

                normal_map.ptr(y)[x] = make_float3(normal.x(), normal.y(), normal.z());
            }

            __global__
            void vanilla_kernel_compute_normal_map(const PtrStepSz<float3> vertex_map, PtrStep<float3> normal_map)
            {
                // for now, this normal calculation is clear enough
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                // disable all the corner cases introduced by gpu allocation.
                if (x < 1 || x >= vertex_map.cols - 1 || y < 1 || y >= vertex_map.rows - 1)
                    return;

                const Vec3fda left(&vertex_map.ptr(y)[x - 1].x);
                const Vec3fda right(&vertex_map.ptr(y)[x + 1].x);
                const Vec3fda upper(&vertex_map.ptr(y - 1)[x].x);
                const Vec3fda lower(&vertex_map.ptr(y + 1)[x].x);

                Vec3fda normal;

                if (left.z() == 0 || right.z() == 0 || upper.z() == 0 || lower.z() == 0)
                    normal = Vec3fda(0.f, 0.f, 0.f);
                else {
                    Vec3fda hor(left.x() - right.x(), left.y() - right.y(), left.z() - right.z());
                    Vec3fda ver(upper.x() - lower.x(), upper.y() - lower.y(), upper.z() - lower.z());

                    normal = hor.cross(ver);
                    normal.normalize();

                    if (normal.z() > 0)
                        normal *= -1;
                }

                normal_map.ptr(y)[x] = make_float3(normal.x(), normal.y(), normal.z());
            }

            void compute_vertex_map(const GpuMat& gpu_mask, const GpuMat& depth_map, GpuMat& vertex_map, const float depth_cutoff,
                                    const CameraParameters cam_params)
            {
                dim3 threads(32, 32);
                dim3 blocks((depth_map.cols + threads.x - 1) / threads.x, (depth_map.rows + threads.y - 1) / threads.y);

                kernel_compute_vertex_map<<<blocks, threads>>>(gpu_mask, depth_map, vertex_map, depth_cutoff, cam_params);

                cudaThreadSynchronize();
            }

            void compute_vertex_map(const GpuMat& depth_map, GpuMat& vertex_map, const float depth_cutoff,
                                    const CameraParameters cam_params)
            {
                dim3 threads(32, 32);
                dim3 blocks((depth_map.cols + threads.x - 1) / threads.x, (depth_map.rows + threads.y - 1) / threads.y);

                kernel_compute_vertex_map<<<blocks, threads>>>(depth_map, vertex_map, depth_cutoff, cam_params);

                cudaThreadSynchronize();
            }

            void compute_normal_map(const GpuMat& vertex_map, GpuMat& normal_map, const CameraParameters cam_params)
            {
                dim3 threads(32, 32);
                dim3 blocks((vertex_map.cols + threads.x - 1) / threads.x,
                            (vertex_map.rows + threads.y - 1) / threads.y);

                kernel_compute_normal_map<<<blocks, threads>>>(vertex_map, normal_map, cam_params);

                cudaThreadSynchronize();
            }

            // use sobel to do image gradient calculation. (to test the great changes in depth image)
            void computeGradient(const cv::cuda::GpuMat& input, cv::cuda::GpuMat& grad, int kernel_size, cv::cuda::Stream& stream) {
                // create sobel filter
                auto sobel_x = cv::cuda::createSobelFilter(input.type(), CV_32F, 1, 0, kernel_size);
                auto sobel_y = cv::cuda::createSobelFilter(input.type(), CV_32F, 0, 1, kernel_size);

                // calculate the gradient by x axis & y axis
                cv::cuda::GpuMat grad_x, grad_y;
                sobel_x->apply(input, grad_x);
                sobel_y->apply(input, grad_y);

                // calculate the magnitude of the depth input. -> by fuse gradient of x axis and y axis
                cv::cuda::magnitude(grad_x, grad_y, grad);
                stream.waitForCompletion();
            }

            // CUDA utils: generate mask by gradient calculation. 
            void generateGradientMask(const cv::cuda::GpuMat& depth, cv::cuda::GpuMat& mask, float gradient_threshold, cv::cuda::Stream& stream) {
                // compute the gradient by fusing x-axis & y-axis gradient
                cv::cuda::GpuMat grad;
                computeGradient(depth, grad, 3, stream);

                // generate the mask
                cv::cuda::GpuMat thresholded;
                cv::cuda::threshold(grad, thresholded, gradient_threshold, 255.0, cv::THRESH_BINARY_INV);

                // convert generated mask to unsigned char -> CV_8UC1
                thresholded.convertTo(mask, CV_8U);
                stream.waitForCompletion();
            }

            // CUDA utils: generate consistency mask
            void generateConsistencyMask(const cv::cuda::GpuMat& depth, cv::cuda::GpuMat& mask, float depth_threshold, cv::cuda::Stream& stream) {
                // create sliding window filter
                auto box_filter = cv::cuda::createBoxFilter(depth.type(), depth.type(), cv::Size(3, 3));

                // compute the mean of neighboring pixels.
                cv::cuda::GpuMat mean_depth;
                box_filter->apply(depth, mean_depth);

                // compute the differences of neighboring depth value
                cv::cuda::GpuMat diff;
                cv::cuda::absdiff(depth, mean_depth, diff);

                // generate threshold of the depth image
                cv::cuda::GpuMat thresholded;
                cv::cuda::threshold(diff, thresholded, depth_threshold, 255.0, cv::THRESH_BINARY_INV);

                // convert the mask to CV_8UC1 mask
                thresholded.convertTo(mask, CV_8U);
                stream.waitForCompletion();
            }

            // CUDA utils: generate IR consistency mask
            void generateIRConsistencyMask(const cv::cuda::GpuMat& depth, const cv::cuda::GpuMat& ir, cv::cuda::GpuMat& mask, float gradient_threshold, cv::cuda::Stream& stream) {
                // calculate the gradient of depth and IR map, respectively.
                cv::cuda::GpuMat depth_grad, ir_grad;
                computeGradient(depth, depth_grad, 3, stream);
                computeGradient(ir, ir_grad, 3, stream);

                // compute the difference of these two gradients.
                cv::cuda::GpuMat diff;
                cv::cuda::absdiff(depth_grad, ir_grad, diff);

                // generate mask.
                cv::cuda::GpuMat thresholded;
                cv::cuda::threshold(diff, thresholded, gradient_threshold, 255.0, cv::THRESH_BINARY_INV);

                // convert to one channel.
                thresholded.convertTo(mask, CV_8U);
                stream.waitForCompletion();
            }

            // caller.
            void generateEdgeMask(cv::cuda::GpuMat& depth_map, 
                      cv::cuda::GpuMat& ir_image, 
                      cv::cuda::GpuMat& existing_mask,
                      float gradient_threshold, // 10-30
                      float consistency_threshold, // 10-50 mm
                      cv::cuda::Stream& stream) {
                // ensure the scale consistency
                CV_Assert(depth_map.size() == ir_image.size());
                CV_Assert(depth_map.type() == CV_32FC1);
                CV_Assert(ir_image.type() == CV_8UC1);
                CV_Assert(existing_mask.size() == depth_map.size());
                CV_Assert(existing_mask.type() == CV_8UC1);

                cv::cuda::GpuMat gradient_mask;
                generateGradientMask(depth_map, gradient_mask, gradient_threshold, stream);

                cv::cuda::GpuMat consistency_mask;
                generateConsistencyMask(depth_map, consistency_mask, consistency_threshold, stream);

                cv::cuda::GpuMat ir_consistency_mask;
                generateIRConsistencyMask(depth_map, ir_image, ir_consistency_mask, gradient_threshold, stream);

                // combine the masks
                cv::cuda::GpuMat combined_mask;
                cv::cuda::bitwise_and(gradient_mask, consistency_mask, combined_mask, cv::noArray(), stream);
                cv::cuda::bitwise_and(combined_mask, ir_consistency_mask, combined_mask, cv::noArray(), stream);
                // write back to existing_mask
                cv::cuda::bitwise_and(existing_mask, combined_mask, existing_mask, cv::noArray(), stream);
                stream.waitForCompletion();
            }
        }
    }
}