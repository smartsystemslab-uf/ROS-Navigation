/*
 * MeanShift.h
 *
 *  Created on: Apr 19, 2016
 *      Author: Streit FJ
 *      email: StreitFr50552@th-nuernberg.de
 */

#ifndef MEANSHIFT_H_
#define MEANSHIFT_H_

class MyMeanShift {
public:
	MyMeanShift() {
	}
	virtual ~MyMeanShift() {
	}

	/****************************************************************************************
	 *
	 * My version of OpenCV Meanshift
	 *
	 ****************************************************************************************/

	inline void MeanShiftFiltering(cv::InputArray _src, cv::OutputArray _dst,
			int sp0, double sr, int max_level, cv::TermCriteria termcrit) {
		const int cn = 3;
		cv::Mat mask0;
		int i, j, level;
#define cdiff(ofs0) (tab[c0-dptr[ofs0]+255] + \
	    tab[c1-dptr[(ofs0)+1]+255] + tab[c2-dptr[(ofs0)+2]+255] >= isr22)
		int tab[768];
		uchar* mask = 0;
		int mstep = 0;
		uchar* dptr;
		int dstep;
		cv::Mat src0 = _src.getMat();

		_dst.create(src0.size(), src0.type());
		cv::Mat dst0 = _dst.getMat();

		int isr2 = sr * sr;
		int isr22 = MAX(isr2, 16);

		std::vector<cv::Mat> src_pyramid(max_level + 1);
		std::vector<cv::Mat> dst_pyramid(max_level + 1);

		if (!(termcrit.type & CV_TERMCRIT_ITER))
			termcrit.maxCount = 5;
		termcrit.maxCount = MAX(termcrit.maxCount, 1);
		termcrit.maxCount = MIN(termcrit.maxCount, 100);
		if (!(termcrit.type & CV_TERMCRIT_EPS))
			termcrit.epsilon = 1.f;
		termcrit.epsilon = MAX(termcrit.epsilon, 0.f);

		for (i = 0; i < 768; i++)
			tab[i] = (i - 255) * (i - 255);

		// 1. construct pyramid
		src_pyramid[0] = src0;
		dst_pyramid[0] = dst0;
		for (level = 1; level <= max_level; level++) {
			src_pyramid[level].create((src_pyramid[level - 1].rows + 1) / 2,
					(src_pyramid[level - 1].cols + 1) / 2,
					src_pyramid[level - 1].type());
			dst_pyramid[level].create(src_pyramid[level].rows,
					src_pyramid[level].cols, src_pyramid[level].type());
			cv::pyrDown(src_pyramid[level - 1], src_pyramid[level],
					src_pyramid[level].size());
			//CV_CALL( cvResize( src_pyramid[level-1], src_pyramid[level], CV_INTER_AREA ));
		}

		mask0.create(src0.rows, src0.cols, CV_8UC1);
		//CV_CALL( submask = (uchar*)cvAlloc( (sp+2)*(sp+2) ));

		// 2. apply meanshift, starting from the pyramid top (i.e. the smallest layer)
		for (level = max_level; level >= 0; level--) {
			cv::Mat src = src_pyramid[level];
			cv::Size size = src.size();
			const uchar* sptr = src.ptr();
			int sstep = (int) src.step;
			int sp = (sp0 / (1 << level));
			sp = MAX(sp, 1);

			if (level < max_level) {
				cv::Size size1 = dst_pyramid[level + 1].size();
				cv::Mat m(size.height, size.width, CV_8UC1, mask0.ptr());
				dstep = (int) dst_pyramid[level + 1].step;
				dptr = dst_pyramid[level + 1].ptr() + dstep + cn;
				mstep = (int) m.step;
				mask = m.ptr() + mstep;
				//cvResize( dst_pyramid[level+1], dst_pyramid[level], CV_INTER_CUBIC );
				cv::pyrUp(dst_pyramid[level + 1], dst_pyramid[level],
						dst_pyramid[level].size());
				m.setTo(cv::Scalar::all(0));

				for (i = 1; i < size1.height - 1;
						i++, dptr += dstep - (size1.width - 2) * 3, mask +=
								mstep * 2) {

					for (j = 1; j < size1.width - 1; j++, dptr += cn) {
						int c0 = dptr[0], c1 = dptr[1], c2 = dptr[2];
						mask[j * 2 - 1] =
								cdiff(
										-3) || cdiff(3) || cdiff(-dstep-3) || cdiff(-dstep) ||
										cdiff(-dstep+3) || cdiff(dstep-3) || cdiff(dstep) || cdiff(dstep+3);
					}
				}

				cv::dilate(m, m, cv::Mat());
				mask = m.ptr();
			}

			dptr = dst_pyramid[level].ptr();
			dstep = (int) dst_pyramid[level].step;

			for (i = 0; i < size.height;
					i++, sptr += sstep - size.width * 3, dptr += dstep
							- size.width * 3, mask += mstep) {
				for (j = 0; j < size.width; j++, sptr += 3, dptr += 3) {
					int x0 = j, y0 = i, x1, y1, iter;
					int c0, c1, c2;

					if (mask && !mask[j])
						continue;

					c0 = sptr[0], c1 = sptr[1], c2 = sptr[2];

					// iterate meanshift procedure

					for (iter = 0; iter < termcrit.maxCount; iter++) {
						const uchar* ptr;
						int x, y, count = 0;
						int minx, miny, maxx, maxy;
						int s0 = 0, s1 = 0, s2 = 0, sx = 0, sy = 0;
						double icount;
						int stop_flag;

						//mean shift: process pixels in window (p-sigmaSp)x(p+sigmaSp)
						minx = cvRound(x0 - sp);
						minx = MAX(minx, 0);
						miny = cvRound(y0 - sp);
						miny = MAX(miny, 0);
						maxx = cvRound(x0 + sp);
						maxx = MIN(maxx, size.width - 1);
						maxy = cvRound(y0 + sp);
						maxy = MIN(maxy, size.height - 1);
						ptr = sptr + (miny - i) * sstep + (minx - j) * 3;

						for (y = miny; y <= maxy;
								y++, ptr += sstep - (maxx - minx + 1) * 3) {
							int row_count = 0;
							x = minx;
#if CV_ENABLE_UNROLLED
							for(; x + 3 <= maxx; x += 4, ptr += 12 )
							{
								int t0 = ptr[0], t1 = ptr[1], t2 = ptr[2];
								if( tab[t0-c0+255] + tab[t1-c1+255] + tab[t2-c2+255] <= isr2 )
								{
									s0 += t0; s1 += t1; s2 += t2;
									sx += x; row_count++;
								}
								t0 = ptr[3], t1 = ptr[4], t2 = ptr[5];
								if( tab[t0-c0+255] + tab[t1-c1+255] + tab[t2-c2+255] <= isr2 )
								{
									s0 += t0; s1 += t1; s2 += t2;
									sx += x+1; row_count++;
								}
								t0 = ptr[6], t1 = ptr[7], t2 = ptr[8];
								if( tab[t0-c0+255] + tab[t1-c1+255] + tab[t2-c2+255] <= isr2 )
								{
									s0 += t0; s1 += t1; s2 += t2;
									sx += x+2; row_count++;
								}
								t0 = ptr[9], t1 = ptr[10], t2 = ptr[11];
								if( tab[t0-c0+255] + tab[t1-c1+255] + tab[t2-c2+255] <= isr2 )
								{
									s0 += t0; s1 += t1; s2 += t2;
									sx += x+3; row_count++;
								}
							}
#endif
							for (; x <= maxx; x++, ptr += 3) {
								int t0 = ptr[0], t1 = ptr[1], t2 = ptr[2];
								if (tab[t0 - c0 + 255] + tab[t1 - c1 + 255]
										+ tab[t2 - c2 + 255] <= isr2) {
									s0 += t0;
									s1 += t1;
									s2 += t2;
									sx += x;
									row_count++;
								}
							}
							count += row_count;
							sy += y * row_count;
						}

						if (count == 0)
							break;

						icount = 1. / count;
						x1 = cvRound(sx * icount);
						y1 = cvRound(sy * icount);
						s0 = cvRound(s0 * icount);
						s1 = cvRound(s1 * icount);
						s2 = cvRound(s2 * icount);

						stop_flag = (x0 == x1 && y0 == y1)
								|| std::abs(x1 - x0) + std::abs(y1 - y0)
										+ tab[s0 - c0 + 255]
										+ tab[s1 - c1 + 255]
										+ tab[s2 - c2 + 255]
										<= termcrit.epsilon;

						x0 = x1;
						y0 = y1;
						c0 = s0;
						c1 = s1;
						c2 = s2;

						if (stop_flag)
							break;
					}

					dptr[0] = (uchar) c0;
					dptr[1] = (uchar) c1;
					dptr[2] = (uchar) c2;
				}

			}
		}
	}
};
#endif /* MEANSHIFT_H_ */
