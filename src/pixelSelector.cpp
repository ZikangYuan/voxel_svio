#include "pixelSelector.h"

pixelSelector::pixelSelector(int w, int h)
{
	random_pattern = new unsigned char[w * h];

	std::srand(3141592);

	for (int i = 0; i < w * h; i++)
		random_pattern[i] = rand() & 0xFF;

	current_potential = 3;

	grad_hist = new int[100 * (1 + w / 32) * (1 + h / 32)];
	ths = new float[(w / 32) * (h / 32) + 100];
	ths_smoothed = new float[(w / 32) * (h / 32) + 100];

	allow_fast = false;
	grad_hist_frame = nullptr;
}

pixelSelector::~pixelSelector()
{
	delete[] random_pattern;
	delete[] grad_hist;
	delete[] ths;
	delete[] ths_smoothed;
}

int computeHistQuantil(int* hist, float below)
{
	int th = hist[0] * below + 0.5f;

	for(int i = 0; i < 90; i++)
	{
		th -= hist[i + 1];
		if(th < 0) return i;
	}

	return 90;
}

void pixelSelector::makeHistsLeft(std::shared_ptr<frame> fh)
{
	grad_hist_frame = fh;
	float* mapmax_0 = fh->abs_squared_grad_left[0];

	int w = wG[0];
	int h = hG[0];

	int w32 = w / 32;
	int h32 = h / 32;
	ths_step = w32;

	for(int y = 0; y < h32; y++)
		for(int x = 0; x < w32; x++)
		{
			float* map_0 = mapmax_0 + 32 * x + 32 * y * w;
			int* hist_0 = grad_hist;

			memset(hist_0, 0, sizeof(int) * 50);

			for(int j = 0; j < 32; j++)
				for(int i = 0; i < 32; i++)
				{
					int it = i + 32 * x;
					int jt = j + 32 * y;

					if(it > w - 2 || jt > h - 2 || it < 1 || jt < 1) continue;

					int g = sqrtf(map_0[i + j * w]);

					if(g > 48) g = 48;

					hist_0[g + 1]++;
					hist_0[0]++;
				}

			ths[x + y * w32] = computeHistQuantil(hist_0, setting_min_grad_hist_cut) + setting_min_grad_hist_add;
		}

	for(int y = 0; y < h32; y++)
		for(int x = 0; x < w32; x++)
		{
			float sum = 0, num = 0;
			if(x > 0)
			{
				if(y > 0)
				{
					num++;
					sum += ths[x - 1 + (y - 1) * w32];
				}

				if(y < h32 - 1)
				{
					num++;
					sum += ths[x - 1 + (y + 1) * w32];
				}

				num++;
				sum += ths[x - 1 + (y) * w32];
			}

			if(x < w32 - 1)
			{
				if(y > 0)
				{
					num++;
					sum += ths[x + 1 + (y - 1) * w32];
				}
				if(y < h32 - 1)
				{
					num++;
					sum += ths[x + 1 + (y + 1) * w32];
				}

				num++;
				sum += ths[x + 1 + (y) * w32];
			}

			if(y > 0)
			{
				num++;
				sum += ths[x + (y - 1) * w32];
			}

			if(y < h32 - 1)
			{
				num++;
				sum += ths[x + (y + 1) * w32];
			}

			num++;
			sum += ths[x + y * w32];

			ths_smoothed[x + y * w32] = (sum / num) * (sum / num);
		}
}

int pixelSelector::pixelSelectionLeft(std::shared_ptr<frame> fh, float density, int recursions_left, float th_factor)
{
	float num_have = 0;
	float num_want = density;
	float quotia;
	int ideal_potential = current_potential;

	// if(fh != grad_hist_frame) makeHistsLeft(fh);
	makeHistsLeft(fh);

	Eigen::Vector3i n = this->selectLeft(fh, current_potential, th_factor);

	num_have = n[0] + n[1] + n[2];
	quotia = num_want / num_have;

	float K = num_have * (current_potential + 1) * (current_potential + 1);
	ideal_potential = sqrtf(K / num_want) - 1;

	if (ideal_potential < 1) ideal_potential = 1;

	if (recursions_left > 0 && quotia > 1.25 && current_potential > 1)
	{
		if(ideal_potential >= current_potential)
			ideal_potential = current_potential - 1;

		current_potential = ideal_potential;
		return pixelSelectionLeft(fh, density, recursions_left - 1, th_factor);
	}
	else if(recursions_left > 0 && quotia < 0.25)
	{
		if(ideal_potential <= current_potential)
			ideal_potential = current_potential + 1;

		current_potential = ideal_potential;
		return pixelSelectionLeft(fh, density, recursions_left - 1, th_factor);
	}

	int num_have_sub = num_have;

	if(quotia < 0.95)
	{
		int wh = wG[0] * hG[0];
		int rn = 0;
		unsigned char char_th = 255*quotia;
		for(int i = 0; i < wh; i++)
		{
			if(fh->selected_pixels_left[i] != 0)
			{
				if(random_pattern[rn] > char_th )
				{
					fh->selected_pixels_left[i] = 0;
					num_have_sub--;
				}
				rn++;
			}
		}
	}

	current_potential = ideal_potential;

	return num_have_sub;
}

Eigen::Vector3i pixelSelector::selectLeft(std::shared_ptr<frame> fh, int pot, float th_factor)
{
	Eigen::Vector3f const * const map_0 = fh->dI_left;

	float* mapmax_0 = fh->abs_squared_grad_left[0];
	float* mapmax_1 = fh->abs_squared_grad_left[1];
	float* mapmax_2 = fh->abs_squared_grad_left[2];

	int w = wG[0];
	int w1 = wG[1];
	int w2 = wG[2];
	int h = hG[0];

	const Eigen::Matrix<float, 2, 1> directions[16] = {
		Eigen::Matrix<float, 2, 1>(0, 1.0000), 
		Eigen::Matrix<float, 2, 1>(0.3827, 0.9239), 
		Eigen::Matrix<float, 2, 1>(0.1951, 0.9808), 
		Eigen::Matrix<float, 2, 1>(0.9239, 0.3827), 
		Eigen::Matrix<float, 2, 1>(0.7071, 0.7071), 
		Eigen::Matrix<float, 2, 1>(0.3827, -0.9239), 
		Eigen::Matrix<float, 2, 1>(0.8315, 0.5556), 
		Eigen::Matrix<float, 2, 1>(0.8315, -0.5556), 
		Eigen::Matrix<float, 2, 1>(0.5556, -0.8315), 
		Eigen::Matrix<float, 2, 1>(0.9808, 0.1951), 
		Eigen::Matrix<float, 2, 1>(0.9239, -0.3827), 
		Eigen::Matrix<float, 2, 1>(0.7071, -0.7071), 
		Eigen::Matrix<float, 2, 1>(0.5556, 0.8315), 
		Eigen::Matrix<float, 2, 1>(0.9808, -0.1951), 
		Eigen::Matrix<float, 2, 1>(1.0000, 0.0000), 
		Eigen::Matrix<float, 2, 1>(0.1951, -0.9808)};

	memset(fh->selected_pixels_left, 0, w * h * sizeof(PixelSelectorStatus));

	float dw1 = setting_grad_down_weight_per_level;
	float dw2 = dw1 * dw1;

	int n3 = 0, n2 = 0, n4 = 0;

	for(int y4 = 0; y4 < h; y4 += (4 * pot))
	{
		for(int x4 = 0; x4 < w; x4 += (4 * pot))
		{
			int my3 = std::min((4 * pot), h - y4);
			int mx3 = std::min((4 * pot), w - x4);
			int best_idx_4 = -1;
			float best_val_4 = 0;

			Eigen::Matrix<float, 2, 1> dir4 = directions[random_pattern[n2] & 0xF];

			for(int y3 = 0; y3 < my3; y3 += (2 * pot))
			{
				for(int x3 = 0; x3 < mx3; x3 += (2 * pot))
				{
					int x34 = x3 + x4;
					int y34 = y3 + y4;
					int my2 = std::min((2 * pot), h - y34);
					int mx2 = std::min((2 * pot), w - x34);
					
					int best_idx_3 = -1;
					float best_val_3 = 0;

					Eigen::Matrix<float, 2, 1> dir3 = directions[random_pattern[n2] & 0xF];

					for(int y2 = 0; y2 < my2; y2 += pot)
					{
						for(int x2 = 0; x2 < mx2; x2 += pot)
						{
							int x234 = x2 + x34;
							int y234 = y2 + y34;
							int my1 = std::min(pot, h - y234);
							int mx1 = std::min(pot, w - x234);
					
							int best_idx_2 = -1;
							float best_val_2 = 0;
				
							Eigen::Matrix<float, 2, 1> dir2 = directions[random_pattern[n2] & 0xF];

							for(int y1 = 0; y1 < my1; y1 += 1)
							{
								for(int x1 = 0; x1 < mx1; x1 += 1)
								{
									assert(x1 + x234 < w);
									assert(y1 + y234 < h);
						
									int idx = x1 + x234 + w * (y1 + y234);
									int xf = x1 + x234;
									int yf = y1 + y234;

									if(xf < 4 || xf >= w - 5 || yf < 4 || yf > h - 4) continue;

									if (fh->mask_left[idx] == true) continue;

									float pixel_TH_0 = ths_smoothed[(xf >> 5) + (yf >> 5) * ths_step];
									float pixel_TH_1 = pixel_TH_0 * dw1;
									float pixel_TH_2 = pixel_TH_1 * dw2;

									float ag_0 = mapmax_0[idx];

									if(ag_0 > pixel_TH_0 * th_factor)
									{
										Eigen::Matrix<float, 2, 1> ag_0_d = map_0[idx].tail<2>();
										float dir_norm = fabsf((float)(ag_0_d.dot(dir2)));

										if(!setting_select_direction_distribution) dir_norm = ag_0;

										if(dir_norm > best_val_2)
										{
											best_val_2 = dir_norm;
											best_idx_2 = idx;
											best_idx_3 = -2;
											best_idx_4 = -2;
										}
									}
						
									if(best_idx_3 == -2) continue;

									float ag_1 = mapmax_1[(int)(xf * 0.5f + 0.25f) + (int)(yf * 0.5f + 0.25f) * w1];

									if(ag_1 > pixel_TH_1 * th_factor)
									{
										Eigen::Matrix<float, 2, 1> ag_0_d = map_0[idx].tail<2>();
										float dir_norm = fabsf((float)(ag_0_d.dot(dir3)));

										if(!setting_select_direction_distribution) dir_norm = ag_1;

										if(dir_norm > best_val_3)
										{
											best_val_3 = dir_norm;
											best_idx_3 = idx;
											best_idx_4 = -2;
										}
									}

									if(best_idx_4 == -2) continue;

									float ag_2 = mapmax_2[(int)(xf * 0.25f + 0.125) + (int)(yf * 0.25f + 0.125) * w2];

									if(ag_2 > pixel_TH_2 * th_factor)
									{
										Eigen::Matrix<float, 2, 1> ag_0_d = map_0[idx].tail<2>();
										float dir_norm = fabsf((float)(ag_0_d.dot(dir4)));

										if(!setting_select_direction_distribution) dir_norm = ag_2;

										if(dir_norm > best_val_4)
										{
											best_val_4 = dir_norm;
											best_idx_4 = idx;
										}
									}
								}
							}

							if(best_idx_2 > 0)
							{
								fh->selected_pixels_left[best_idx_2] = 1;
								best_val_3 = 1e10;
								n2++;
							}
						}
					}

					if(best_idx_3 > 0)
					{
						fh->selected_pixels_left[best_idx_3] = 2;
						best_val_4 = 1e10;
						n3++;
					}
				}

				if(best_idx_4 > 0)
				{
					fh->selected_pixels_left[best_idx_4] = 4;
					n4++;
				}
			}
		}
	}

	return Eigen::Vector3i(n2, n3, n4);
}

void pixelSelector::makeHistsRight(std::shared_ptr<frame> fh)
{
	grad_hist_frame = fh;
	float* mapmax_0 = fh->abs_squared_grad_right[0];

	int w = wG[0];
	int h = hG[0];

	int w32 = w / 32;
	int h32 = h / 32;
	ths_step = w32;

	for(int y = 0; y < h32; y++)
		for(int x = 0; x < w32; x++)
		{
			float* map_0 = mapmax_0 + 32 * x + 32 * y * w;
			int* hist_0 = grad_hist;

			memset(hist_0, 0, sizeof(int) * 50);

			for(int j = 0; j < 32; j++)
				for(int i = 0; i < 32; i++)
				{
					int it = i + 32 * x;
					int jt = j + 32 * y;

					if(it > w - 2 || jt > h - 2 || it < 1 || jt < 1) continue;

					int g = sqrtf(map_0[i + j * w]);

					if(g > 48) g = 48;

					hist_0[g + 1]++;
					hist_0[0]++;
				}

			ths[x + y * w32] = computeHistQuantil(hist_0, setting_min_grad_hist_cut) + setting_min_grad_hist_add;
		}

	for(int y = 0; y < h32; y++)
		for(int x = 0; x < w32; x++)
		{
			float sum = 0, num = 0;

			if(x > 0)
			{
				if(y > 0)
				{
					num++;
					sum += ths[x - 1 + (y - 1) * w32];
				}

				if(y < h32 - 1)
				{
					num++;
					sum += ths[x - 1 + (y + 1) * w32];
				}

				num++;
				sum += ths[x - 1 + (y) * w32];
			}

			if(x < w32 - 1)
			{
				if(y > 0)
				{
					num++;
					sum += ths[x + 1 + (y - 1) * w32];
				}
				if(y < h32 - 1)
				{
					num++;
					sum += ths[x + 1 + (y + 1) * w32];
				}

				num++;
				sum += ths[x + 1 + (y) * w32];
			}

			if(y > 0)
			{
				num++;
				sum += ths[x + (y - 1) * w32];
			}

			if(y < h32 - 1)
			{
				num++;
				sum += ths[x + (y + 1) * w32];
			}

			num++;
			sum += ths[x + y * w32];

			ths_smoothed[x + y * w32] = (sum / num) * (sum / num);
		}
}

int pixelSelector::pixelSelectionRight(std::shared_ptr<frame> fh, float density, int recursions_left, float th_factor)
{
	float num_have = 0;
	float num_want = density;
	float quotia;
	int ideal_potential = current_potential;

	// if(fh != grad_hist_frame) makeHistsRight(fh);
	makeHistsRight(fh);

	Eigen::Vector3i n = this->selectRight(fh, current_potential, th_factor);

	num_have = n[0] + n[1] + n[2];
	quotia = num_want / num_have;

	float K = num_have * (current_potential + 1) * (current_potential + 1);
	ideal_potential = sqrtf(K / num_want) - 1;

	if (ideal_potential < 1) ideal_potential = 1;

	if (recursions_left > 0 && quotia > 1.25 && current_potential > 1)
	{
		if(ideal_potential >= current_potential)
			ideal_potential = current_potential - 1;

		current_potential = ideal_potential;
		return pixelSelectionRight(fh, density, recursions_left - 1, th_factor);
	}
	else if (recursions_left > 0 && quotia < 0.25)
	{
		if(ideal_potential <= current_potential)
			ideal_potential = current_potential + 1;

		current_potential = ideal_potential;
		return pixelSelectionRight(fh, density, recursions_left - 1, th_factor);
	}

	int num_have_sub = num_have;

	if (quotia < 0.95)
	{
		int wh = wG[0] * hG[0];
		int rn = 0;
		unsigned char char_th = 255*quotia;
		for (int i = 0; i < wh; i++)
		{
			if (fh->selected_pixels_right[i] != 0)
			{
				if (random_pattern[rn] > char_th)
				{
					fh->selected_pixels_right[i] = 0;
					num_have_sub--;
				}
				rn++;
			}
		}
	}

	current_potential = ideal_potential;

	return num_have_sub;
}

Eigen::Vector3i pixelSelector::selectRight(std::shared_ptr<frame> fh, int pot, float th_factor)
{
	Eigen::Vector3f const * const map_0 = fh->dI_right;

	float* mapmax_0 = fh->abs_squared_grad_right[0];
	float* mapmax_1 = fh->abs_squared_grad_right[1];
	float* mapmax_2 = fh->abs_squared_grad_right[2];

	int w = wG[0];
	int w1 = wG[1];
	int w2 = wG[2];
	int h = hG[0];

	const Eigen::Matrix<float, 2, 1> directions[16] = {
		Eigen::Matrix<float, 2, 1>(0, 1.0000), 
		Eigen::Matrix<float, 2, 1>(0.3827, 0.9239), 
		Eigen::Matrix<float, 2, 1>(0.1951, 0.9808), 
		Eigen::Matrix<float, 2, 1>(0.9239, 0.3827), 
		Eigen::Matrix<float, 2, 1>(0.7071, 0.7071), 
		Eigen::Matrix<float, 2, 1>(0.3827, -0.9239), 
		Eigen::Matrix<float, 2, 1>(0.8315, 0.5556), 
		Eigen::Matrix<float, 2, 1>(0.8315, -0.5556), 
		Eigen::Matrix<float, 2, 1>(0.5556, -0.8315), 
		Eigen::Matrix<float, 2, 1>(0.9808, 0.1951), 
		Eigen::Matrix<float, 2, 1>(0.9239, -0.3827), 
		Eigen::Matrix<float, 2, 1>(0.7071, -0.7071), 
		Eigen::Matrix<float, 2, 1>(0.5556, 0.8315), 
		Eigen::Matrix<float, 2, 1>(0.9808, -0.1951), 
		Eigen::Matrix<float, 2, 1>(1.0000, 0.0000), 
		Eigen::Matrix<float, 2, 1>(0.1951, -0.9808)};

	memset(fh->selected_pixels_right, 0, w * h * sizeof(PixelSelectorStatus));

	float dw1 = setting_grad_down_weight_per_level;
	float dw2 = dw1 * dw1;

	int n3 = 0, n2 = 0, n4 = 0;

	for(int y4 = 0; y4 < h; y4 += (4 * pot))
	{
		for(int x4 = 0; x4 < w; x4 += (4 * pot))
		{
			int my3 = std::min((4 * pot), h - y4);
			int mx3 = std::min((4 * pot), w - x4);
			int best_idx_4 = -1;
			float best_val_4 = 0;

			Eigen::Matrix<float, 2, 1> dir4 = directions[random_pattern[n2] & 0xF];

			for(int y3 = 0; y3 < my3; y3 += (2 * pot))
			{
				for(int x3 = 0; x3 < mx3; x3 += (2 * pot))
				{
					int x34 = x3 + x4;
					int y34 = y3 + y4;
					int my2 = std::min((2 * pot), h - y34);
					int mx2 = std::min((2 * pot), w - x34);
					
					int best_idx_3 = -1;
					float best_val_3 = 0;

					Eigen::Matrix<float, 2, 1> dir3 = directions[random_pattern[n2] & 0xF];

					for(int y2 = 0; y2 < my2; y2 += pot)
					{
						for(int x2 = 0; x2 < mx2; x2 += pot)
						{
							int x234 = x2 + x34;
							int y234 = y2 + y34;
							int my1 = std::min(pot, h - y234);
							int mx1 = std::min(pot, w - x234);
					
							int best_idx_2 = -1;
							float best_val_2 = 0;
				
							Eigen::Matrix<float, 2, 1> dir2 = directions[random_pattern[n2] & 0xF];

							for(int y1 = 0; y1 < my1; y1 += 1)
							{
								for(int x1 = 0; x1 < mx1; x1 += 1)
								{
									assert(x1 + x234 < w);
									assert(y1 + y234 < h);
						
									int idx = x1 + x234 + w * (y1 + y234);
									int xf = x1 + x234;
									int yf = y1 + y234;

									if (xf < 4 || xf >= w - 5 || yf < 4 || yf > h - 4) continue;

									if (fh->mask_right[idx] == true) continue;

									float pixel_TH_0 = ths_smoothed[(xf >> 5) + (yf >> 5) * ths_step];
									float pixel_TH_1 = pixel_TH_0 * dw1;
									float pixel_TH_2 = pixel_TH_1 * dw2;

									float ag_0 = mapmax_0[idx];

									if(ag_0 > pixel_TH_0 * th_factor)
									{
										Eigen::Matrix<float, 2, 1> ag_0_d = map_0[idx].tail<2>();
										float dir_norm = fabsf((float)(ag_0_d.dot(dir2)));

										if(!setting_select_direction_distribution) dir_norm = ag_0;

										if(dir_norm > best_val_2)
										{
											best_val_2 = dir_norm;
											best_idx_2 = idx;
											best_idx_3 = -2;
											best_idx_4 = -2;
										}
									}
						
									if(best_idx_3 == -2) continue;

									float ag_1 = mapmax_1[(int)(xf * 0.5f + 0.25f) + (int)(yf * 0.5f + 0.25f) * w1];

									if(ag_1 > pixel_TH_1 * th_factor)
									{
										Eigen::Matrix<float, 2, 1> ag_0_d = map_0[idx].tail<2>();
										float dir_norm = fabsf((float)(ag_0_d.dot(dir3)));

										if(!setting_select_direction_distribution) dir_norm = ag_1;

										if(dir_norm > best_val_3)
										{
											best_val_3 = dir_norm;
											best_idx_3 = idx;
											best_idx_4 = -2;
										}
									}

									if(best_idx_4 == -2) continue;

									float ag_2 = mapmax_2[(int)(xf * 0.25f + 0.125) + (int)(yf * 0.25f + 0.125) * w2];

									if(ag_2 > pixel_TH_2 * th_factor)
									{
										Eigen::Matrix<float, 2, 1> ag_0_d = map_0[idx].tail<2>();
										float dir_norm = fabsf((float)(ag_0_d.dot(dir4)));

										if(!setting_select_direction_distribution) dir_norm = ag_2;

										if(dir_norm > best_val_4)
										{
											best_val_4 = dir_norm;
											best_idx_4 = idx;
										}
									}
								}
							}

							if(best_idx_2 > 0)
							{
								fh->selected_pixels_right[best_idx_2] = 1;
								best_val_3 = 1e10;
								n2++;
							}
						}
					}

					if(best_idx_3 > 0)
					{
						fh->selected_pixels_right[best_idx_3] = 2;
						best_val_4 = 1e10;
						n3++;
					}
				}

				if(best_idx_4 > 0)
				{
					fh->selected_pixels_right[best_idx_4] = 4;
					n4++;
				}
			}
		}
	}

	return Eigen::Vector3i(n2, n3, n4);
}
