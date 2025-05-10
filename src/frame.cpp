#include "frame.h"

void frame::release()
{
	delete[] mask_left;
	delete[] mask_right;

	delete[] selected_pixels_left;
	delete[] selected_pixels_right;

	for(int i = 0; i < pyr_levels_used; i++)
	{
		delete[] dIp_left[i];
		delete[] dIp_right[i];

		delete[] abs_squared_grad_left[i];
		delete[] abs_squared_grad_right[i];
	}

	is_invalid = true;
}

void frame::makeImageLeft(unsigned char* color, unsigned char* mask, std::shared_ptr<gammaPixel> gammaPixel_ptr, double timestamp_)
{
	timestamp = timestamp_;

	for(int i = 0; i < pyr_levels_used; i++)
	{
		dIp_left[i] = new Eigen::Vector3f[wG[i] * hG[i]];
		abs_squared_grad_left[i] = new float[wG[i] * hG[i]];

		if (i == 0) mask_left = new bool[wG[i] * hG[i]];
	}

	dI_left = dIp_left[0];

	int w = wG[0];
	int h = hG[0];

	for(int i = 0; i < w * h; i++)
	{
		dI_left[i][0] = (float)color[i];
		mask_left[i] = mask[i] > 0 ? true : false;
	}

	for(int lvl = 0; lvl < pyr_levels_used; lvl++)
	{
		int wl = wG[lvl], hl = hG[lvl];
		Eigen::Vector3f* dI_l = dIp_left[lvl];
		float* dabs_l = abs_squared_grad_left[lvl];

		if (lvl > 0)
		{
			int lvlm1 = lvl - 1;
			int wlm1 = wG[lvlm1];

			Eigen::Vector3f* dI_lm = dIp_left[lvlm1];

			for(int y = 0; y < hl; y++)
			{
				for(int x = 0; x < wl; x++)
				{
					dI_l[x + y * wl][0] = 0.25f * (dI_lm[2 * x + 2 * y *wlm1][0] + dI_lm[2 * x + 1 + 2 * y * wlm1][0] 
						+ dI_lm[2 * x + 2 * y * wlm1 + wlm1][0] + dI_lm[2 * x + 1 + 2 * y * wlm1 + wlm1][0]);
				}
			}
		}

		for(int idx = wl; idx < wl * (hl - 1); idx++)
		{
			float dx = 0.5f * (dI_l[idx + 1][0] - dI_l[idx - 1][0]);
			float dy = 0.5f * (dI_l[idx + wl][0] - dI_l[idx - wl][0]);

			if(!std::isfinite(dx)) dx = 0;
			if(!std::isfinite(dy)) dy = 0;

			dI_l[idx][1] = dx;
			dI_l[idx][2] = dy;

			dabs_l[idx] = dx * dx + dy * dy;

			if(setting_gamma_pixel_selection == 1 && gammaPixel_ptr != nullptr)
			{
				float gw = gammaPixel_ptr->getBGradOnly((float)(dI_l[idx][0]));
				dabs_l[idx] *= gw * gw;
			}
		}
	}

	selected_pixels_left = new float[wG[0] * hG[0]];
}

void frame::makeImageRight(unsigned char* color, unsigned char* mask, std::shared_ptr<gammaPixel> gammaPixel_ptr, double timestamp_)
{
	timestamp = timestamp_;

	for(int i = 0; i < pyr_levels_used; i++)
	{
		dIp_right[i] = new Eigen::Vector3f[wG[i] * hG[i]];
		abs_squared_grad_right[i] = new float[wG[i] * hG[i]];

		if (i == 0) mask_right = new bool[wG[i] * hG[i]];
	}

	dI_right = dIp_right[0];

	int w = wG[0];
	int h = hG[0];

	for(int i = 0; i < w * h; i++)
	{
		dI_right[i][0] = (float)color[i];
		mask_right[i] = mask[i] > 0 ? true : false;
	}

	for(int lvl = 0; lvl < pyr_levels_used; lvl++)
	{
		int wl = wG[lvl], hl = hG[lvl];
		Eigen::Vector3f* dI_l = dIp_right[lvl];
		float* dabs_l = abs_squared_grad_right[lvl];

		if (lvl > 0)
		{
			int lvlm1 = lvl - 1;
			int wlm1 = wG[lvlm1];

			Eigen::Vector3f* dI_lm = dIp_right[lvlm1];

			for(int y = 0; y < hl; y++)
			{
				for(int x = 0; x < wl; x++)
				{
					dI_l[x + y * wl][0] = 0.25f * (dI_lm[2 * x + 2 * y *wlm1][0] + dI_lm[2 * x + 1 + 2 * y * wlm1][0] 
						+ dI_lm[2 * x + 2 * y * wlm1 + wlm1][0] + dI_lm[2 * x + 1 + 2 * y * wlm1 + wlm1][0]);
				}
			}
		}

		for(int idx = wl; idx < wl * (hl - 1); idx++)
		{
			float dx = 0.5f * (dI_l[idx + 1][0] - dI_l[idx - 1][0]);
			float dy = 0.5f * (dI_l[idx + wl][0] - dI_l[idx - wl][0]);

			if(!std::isfinite(dx)) dx = 0;
			if(!std::isfinite(dy)) dy = 0;

			dI_l[idx][1] = dx;
			dI_l[idx][2] = dy;

			dabs_l[idx] = dx * dx + dy * dy;

			if(setting_gamma_pixel_selection == 1 && gammaPixel_ptr != nullptr)
			{
				float gw = gammaPixel_ptr->getBGradOnly((float)(dI_l[idx][0]));
				dabs_l[idx] *= gw * gw;
			}
		}
	}

	selected_pixels_right = new float[wG[0] * hG[0]];
}