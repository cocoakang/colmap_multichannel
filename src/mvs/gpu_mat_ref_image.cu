// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "mvs/gpu_mat_ref_image.h"

#include <iostream>

#include "util/cudacc.h"
#define TMP_DEPTH 3
#define MAX_DEPTH 70
namespace colmap {
	namespace mvs {
		namespace {

			texture<uint8_t, cudaTextureType2DLayered, cudaReadModeNormalizedFloat> image_texture;

			__global__ void FilterKernel(GpuMat<uint8_t> image, GpuMat<float> sum_image,
				GpuMat<float> squared_sum_image,
				const int window_radius, const int window_step,
				const float sigma_spatial,
				const float sigma_color,
				const int data_depth) {
				const size_t row = blockDim.y * blockIdx.y + threadIdx.y;
				const size_t col = blockDim.x * blockIdx.x + threadIdx.x;
				if (row >= image.GetHeight() || col >= image.GetWidth()) {
					return;
				}

				BilateralWeightComputer bilateral_weight_computer(sigma_spatial, sigma_color);

				//const float center_color = tex2D(image_texture, col, row);
				//TODO-x: now is grey
				//float center_color_aver = 0;
				//for (int i = 0; i < data_depth; ++i)
				//	center_color_aver += tex2DLayered(image_texture, col, row, i);
				//center_color_aver /= data_depth;
				float center_color[MAX_DEPTH];
				for (int i = 0; i < data_depth; ++i)
					center_color[i] = tex2DLayered(image_texture, col, row, i);


				//float color_sum = 0.0f;
				//float color_squared_sum = 0.0f;
				//NEW
				float bilateral_weight_sum = 0.0f;

				float color_sum[MAX_DEPTH];
				float color_squared_sum[MAX_DEPTH];
				for (int i = 0; i < TMP_DEPTH; ++i)
				{
					color_sum[i] = 0;
					color_squared_sum[i] = 0;
				}

				for (int window_row = -window_radius; window_row <= window_radius;
					window_row += window_step) {
					for (int window_col = -window_radius; window_col <= window_radius;
						window_col += window_step) {
						//const float color =
						//	tex2D(image_texture, col + window_col, row + window_row);
						//float color_aver = 0;
						//for (int i = 0; i < data_depth; ++i)
						//	color_aver += tex2DLayered(image_texture, col + window_col, row + window_row, i);
						//color_aver /= data_depth;

						//NEW
						float color[MAX_DEPTH];
						for (int i = 0; i < TMP_DEPTH; ++i)
							color[i] = tex2DLayered(image_texture, col + window_col, row + window_row, i);

						const float bilateral_weight = bilateral_weight_computer.Compute(
							window_row, window_col, center_color, color, TMP_DEPTH); //NEW

						//color_sum += bilateral_weight * color;
						//color_squared_sum += bilateral_weight * color * color;
						bilateral_weight_sum += bilateral_weight;
						//NEW
						for (int i = 0; i < TMP_DEPTH; ++i)
						{
							color_sum[i] += bilateral_weight * color[i];
							color_squared_sum[i] += bilateral_weight * color[i] * color[i];
						}
					}
				}

				//color_sum /= bilateral_weight_sum;
				//color_squared_sum /= bilateral_weight_sum;
				//NEW
				for (int i = 0; i < TMP_DEPTH; ++i)
				{
					color_sum[i] /= bilateral_weight_sum;
					color_squared_sum[i] /= bilateral_weight_sum;
				}


				//image.Set(row, col, static_cast<uint8_t>(255.0f * center_color));//CHANGED
				//sum_image.Set(row, col, color_sum);
				//squared_sum_image.Set(row, col, color_squared_sum);
				//NEW
				for (int i = 0; i < data_depth; ++i)
				{
					image.Set(row, col, i, static_cast<uint8_t>(255.0f * tex2DLayered(image_texture, col, row, i)));
					sum_image.Set(row, col, i, color_sum[i]);
					squared_sum_image.Set(row, col, i, color_squared_sum[i]);
				}
			}

		}  // namespace

		GpuMatRefImage::GpuMatRefImage(const size_t width, const size_t height, const size_t depth)
			: height_(height), width_(width), depth_(depth) {
			image.reset(new GpuMat<uint8_t>(width, height, depth));
			sum_image.reset(new GpuMat<float>(width, height, depth)); //NEW
			squared_sum_image.reset(new GpuMat<float>(width, height, depth));//NEW
		}

		void GpuMatRefImage::Filter(const uint8_t* image_data,
			const size_t window_radius,
			const size_t window_step, const float sigma_spatial,
			const float sigma_color) {
			CudaArrayWrapper<uint8_t> image_array(width_, height_, depth_);
			image_array.CopyToDevice(image_data);
			image_texture.addressMode[0] = cudaAddressModeBorder;  //out of address return 0
			image_texture.addressMode[1] = cudaAddressModeBorder;
			image_texture.addressMode[2] = cudaAddressModeBorder;
			// returned value is the texel whose texture coordinates are the closest to
			// the input texture coordinates
			image_texture.filterMode = cudaFilterModePoint;
			image_texture.normalized = false;

			const dim3 block_size(kBlockDimX, kBlockDimY);
			const dim3 grid_size((width_ - 1) / block_size.x + 1,
				(height_ - 1) / block_size.y + 1);

			CUDA_SAFE_CALL(cudaBindTextureToArray(image_texture, image_array.GetPtr()));
			FilterKernel << <grid_size, block_size >> >(
				*image, *sum_image, *squared_sum_image, window_radius, window_step,
				sigma_spatial, sigma_color, depth_);
			CUDA_SYNC_AND_CHECK();
			CUDA_SAFE_CALL(cudaUnbindTexture(image_texture));
		}

	}  // namespace mvs
}  // namespace colmap
