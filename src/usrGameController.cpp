#include "usrGameController.h"
#include <queue>
#include <iostream>
#include <fstream>
#include <opencv2/nonfree/nonfree.hpp>//SIFT
#include <opencv2/legacy/legacy.hpp>//BFMatch暴力匹配
#include <math.h>
#ifdef VIA_OPENCV
//构造与初始化

//bool click = false;

////////////////////////////////////////////////////////////////////////////////////////////
//操作机械臂以及操作尝试
struct arr{
	int maxnum, max_row, max_col;
	int a[4][4];
	int counter[11];
};

arr num, tmpr, tmpl, tmpd, tmpu, tmp2ndd,tmp2ndu,tmp3rdu,tmp3rdd,tmp3rdur,tmp3rddr;
int skip_pic= 0;
bool up_done = 0, left_done = 0;

void delay(){
	for (int i = -999999; i< 99999999; i++);
	return;
}

void usrGameController::leftmove(){
	device->comMoveTo(55,0);
	delay();
	device->comHitDown();
	delay();
	device->comMoveTo(35,0);
	device->comHitUp();
	delay();
	device->comMoveTo(0, 0);
	for (int i =0; i<10; i++)delay();
	return;
}

void usrGameController::rightmove(){
	device->comMoveTo(55,0);
	delay();
	device->comHitDown();
	delay();
	device->comMoveTo(75,0);
	device->comHitUp();
	delay();
	device->comMoveTo(0, 0);
	for (int i = 0; i<10; i++)delay();
	return;
}

void usrGameController::upmove(){
	device->comMoveTo(55, 0);
	delay();
	device->comHitDown();
	delay();
	device->comMoveTo(55,-20);
	device->comHitUp();
	delay();
	device->comMoveTo(0, 0);
	for (int i = 0; i<10; i++)delay();
	return;
}

void usrGameController::downmove(){
	device->comMoveTo(55, 0);
	delay();
	device->comHitDown();
	delay();
	device->comMoveTo(55, 20);
	device->comHitUp();
	delay();
	device->comMoveTo(0, 0);
	for (int i = 0; i<10; i++)delay();
	return;
}

void count(arr &num){
	for (int i = 0; i < 11; i++){
		num.counter[i] = 0;
	}
	for (int i = 0; i<4; i++){
		for (int j = 0; j<4; j++){
			switch (num.a[i][j]){
			case 2048:
				num.counter[10] += 1;
			case 1024:
				num.counter[9] += 1;
			case 512:
				num.counter[8] += 1;
			case 256:
				num.counter[7] += 1;
			case 128:
				num.counter[6] += 1;
			case 64:
				num.counter[5] += 1;
			case 32:
				num.counter[4] += 1;
			case 16:
				num.counter[3] += 1;
			case 8:
				num.counter[2] += 1;
			case 4:
				num.counter[1] += 1;
			case 2:
				num.counter[0] += 1;
			}
		}
	}
	return;
}

void findmax(arr &num){
	num.maxnum = 0;
	num.max_row = 0;
	num.max_col = 0;
	for (int i = 0; i<4; i++){
		for (int j = 0; j<4; j++){
			if (num.a[i][j] >= num.maxnum){
				num.maxnum = num.a[i][j];
				num.max_row = i;
				num.max_col = j;
			}
		}
	}
	return;
};

bool compare(arr &num, arr &tmp){
	bool flag = 1;
	for (int i = 0; i<4; i++){
		for (int j = 0; j<4; j++){
			if (num.a[i][j] != tmp.a[i][j]){ flag = 0; }
		}
		//if (a[i][j] != b[i][j]){ break; }
	}
	return flag;
};

bool cntcmp(arr &tmp0, arr &tmp2){
	bool flag = 0;
	for (int i = 10; i >= 0; i--){
		if (tmp0.counter[i] > tmp2.counter[i]){
			flag = 1; break;
		}
		if (tmp0.counter[i] < tmp2.counter[i]){
			break;
		}
	}
	return flag;
}

bool max_in_corner(arr &num){
	return ((num.max_row == 3 && num.max_col == 3)) ? 1 : 0;
};

void tryleft(arr &num, arr &tmpl){
	int i, j, k;
	for (i = 0; i<4; ++i){
		for (j = 0; j<4; ++j){
			tmpl.a[i][j] = num.a[i][j];
		}
	}
	for (i = 0; i<4; ++i){
		k = 0;
		for (j = 0; j<3; ++j){
			if (tmpl.a[i][j] == 0){
				for (int n = j; n<3; n++){ tmpl.a[i][n] = tmpl.a[i][n + 1]; tmpl.a[i][n + 1] = 0; }//向左移位
				j>0 ? j-- : j = -1;//停步再次判断
				if (k < 4) k++; else break;//跳出零行
			}
		}
	}
	for (i = 0; i<4; ++i){
		for (j = 0; j<3; ++j){
			if (tmpl.a[i][j] == tmpl.a[i][j + 1] && tmpl.a[i][j] != 0){
				tmpl.a[i][j] *= 2;//合并
				tmpl.a[i][j + 1] = 0;
				for (int n = j + 1; n<3; n++){ tmpl.a[i][n] = tmpl.a[i][n + 1]; tmpl.a[i][n + 1] = 0; }//向左移位
			}
		}
	}
	findmax(tmpl);
	count(tmpl);
	return;
};

void tryright(arr &num, arr &tmpr){
	int i, j, k;
	for (i = 0; i<4; ++i){
		for (j = 0; j<4; ++j){
			tmpr.a[i][j] = num.a[i][j];
		}
	}
	for (i = 0; i<4; ++i){
		k = 0;
		for (j = 3; j>0; j--){
			if (tmpr.a[i][j] == 0){
				for (int n = j; n>0; n--){ tmpr.a[i][n] = tmpr.a[i][n - 1]; tmpr.a[i][n - 1] = 0; }//向右移位
				j == 3 ? j = 4 : j++;//停步再次判断
				if (k < 4) k++; else break;//跳出零行
			}
		}
	}
	for (i = 0; i<4; ++i){
		for (j = 3; j>0; j--){
			if (tmpr.a[i][j] == tmpr.a[i][j - 1] && tmpr.a[i][j] != 0){
				tmpr.a[i][j] *= 2;//合并
				tmpr.a[i][j - 1] = 0;
				for (int n = j - 1; n>0; n--){ tmpr.a[i][n] = tmpr.a[i][n - 1]; tmpr.a[i][n - 1] = 0; }//向右移位
			}
		}
	}
	findmax(tmpr);
	count(tmpr);
	return;
};

void tryup(arr &num, arr &tmpu){
	int i, j, k;
	for (i = 0; i<4; ++i){
		for (j = 0; j<4; ++j){
			tmpu.a[i][j] = num.a[i][j];
		}
	}
	for (j = 0; j<4; ++j){
		k = 0;
		for (i = 0; i<3; ++i){
			if (tmpu.a[i][j] == 0){
				for (int n = i; n<3; n++){ tmpu.a[n][j] = tmpu.a[n + 1][j]; tmpu.a[n + 1][j] = 0; }//向上移位
				i>0 ? i-- : i = -1;//停步再次判断
				if (k < 4) k++; else break;//跳出零行
			}
		}
	}
	for (j = 0; j<4; ++j){
		for (i = 0; i<3; ++i){
			if (tmpu.a[i][j] == tmpu.a[i + 1][j] && tmpu.a[i][j] != 0){
				tmpu.a[i][j] *= 2;//合并
				tmpu.a[i + 1][j] = 0;
				for (int n = i + 1; n<3; n++){ tmpu.a[n][j] = tmpu.a[n + 1][j]; tmpu.a[n + 1][j] = 0; }//向上移位
			}
		}
	}
	findmax(tmpu);
	count(tmpu);
	return;
};

void trydown(arr &num, arr &tmpd){
	int i, j, k;
	for (i = 0; i<4; ++i){
		for (j = 0; j<4; ++j){
			tmpd.a[i][j] = num.a[i][j];
		}
	}
	for (j = 0; j<4; ++j){
		k = 0;
		for (i = 3; i>0; i--){
			if (tmpd.a[i][j] == 0){
				for (int n = i; n>0; n--){ tmpd.a[n][j] = tmpd.a[n - 1][j]; tmpd.a[n - 1][j] = 0; }//向下移位
				i == 3 ? i = 4 : i++;//停步再次判断
				if (k < 4) k++; else break;//跳出零行
			}
		}
	}
	for (j = 0; j<4; ++j){
		for (i = 3; i>0; i--){
			if (tmpd.a[i][j] == tmpd.a[i - 1][j] && tmpd.a[i][j] != 0){
				tmpd.a[i][j] *= 2;//合并
				tmpd.a[i - 1][j] = 0;
				for (int n = i - 1; n>0; n--){ tmpd.a[n][j] = tmpd.a[n - 1][j]; tmpd.a[n - 1][j] = 0; }//向下移位
			}
		}
	}
	findmax(tmpd);
	count(tmpd);
	return;
};
//////////////////////////////////////////////////////////////////////////////////
void usrGameController::trainSVM(){
	cv::HOGDescriptor hog(cv::Size(32, 32), cv::Size(8, 8), cv::Size(4, 4), cv::Size(4, 4), 10);
	int DescriptorDim;

	cv::Mat sampleFeatureMat;
	cv::Mat sampleLabelMat;

	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 3000, 1e-6);
	

	std::string ImgName;//图片名(绝对路径)

	std::ifstream finPos("img.txt");//正样本图片的文件名列表

	int numLine = 114;
	for (int num = 0; num < numLine && getline(finPos, ImgName); num++)
	{
		std::cout << "Now processing original positive image: " << ImgName << std::endl;

		cv::Mat src = cv::imread(ImgName);//读取图片
		cv::imshow("src", src);
		std::vector<float> descriptors;//HOG描述子向量
		hog.compute(src, descriptors, cv::Size(4, 4));//计算HOG描述子

		if (0 == num)
		{
			DescriptorDim = descriptors.size();
			//初始化特征向量组成的矩阵
			sampleFeatureMat = cv::Mat::zeros(numLine, DescriptorDim, CV_32FC1);
			//初始化训练样本的类别向量
			sampleLabelMat = cv::Mat::zeros(numLine, 1, CV_32SC1);
		}
		////将计算好的HOG描述子复制到样本特征矩阵sampleFeatureMat
		for (int i = 0; i < DescriptorDim; i++){
			sampleFeatureMat.at<float>(num, i) = descriptors[i];//第num个样本的特征向量中的第i个元素
		}
		if (num < 19){
			sampleLabelMat.at<int>(num, 0) = 0;
		}
		else if (num < 29){
			sampleLabelMat.at<int>(num, 0) = 2;
		}
		else if (num < 39){
			sampleLabelMat.at<int>(num, 0) = 4;
		}
		else if (num < 49){
			sampleLabelMat.at<int>(num, 0) = 8;
		}
		else if (num < 59){
			sampleLabelMat.at<int>(num, 0) = 16;
		}
		else if (num < 69){
			sampleLabelMat.at<int>(num, 0) = 32;
		}
		else if (num < 79){
			sampleLabelMat.at<int>(num, 0) = 64;
		}
		else if (num < 89){
			sampleLabelMat.at<int>(num, 0) = 128;
		}
		else if (num < 99){
			sampleLabelMat.at<int>(num, 0) = 256;
		}
		else if (num < 109){
			sampleLabelMat.at<int>(num, 0) = 512;
		}
		else{
			sampleLabelMat.at<int>(num, 0) = 1024;
		}
		std::cout << sampleLabelMat.at<int>(num, 0) << std::endl;

	}
	finPos.close();
	std::cout << "Starting training..." << std::endl;
	CvSVM SVM;
	SVM.train(sampleFeatureMat, sampleLabelMat, cv::Mat(), cv::Mat(), params);
	std::cout << "Finishing training..." << std::endl;
	//将训练好的SVM模型保存为xml文件
	SVM.save("SVM_HOG10.xml");
	std::cout << "Finishing save file..." << std::endl;
	std::cout << SVM.get_support_vector_count() << std::endl; 
	std::cout << SVM.get_var_count() << std::endl; 

}

usrGameController::usrGameController(void* qtCD)
{
	//trainSVM(); 训练SVM
	svm.load("SVM_HOG7.xml");
	qDebug() << "usrGameController online.";
	device = new deviceCyberDip(qtCD);//设备代理类
	cv::namedWindow(WIN_NAME);
	cv::setMouseCallback(WIN_NAME, mouseCallback, (void*)&(argM));
}

//析构
usrGameController::~usrGameController()
{
	cv::destroyAllWindows();
	if (device != nullptr)
	{
		delete device;
	}
	qDebug() << "usrGameController offline.";
}

//处理图像 
int usrGameController::usrProcessImage(cv::Mat& img)
{
	skip_pic++;
	int board[4][4] = {};

	cv::Size imgSize(img.cols, img.rows - UP_CUT);
	if (imgSize.height <= 0 || imgSize.width <= 0)
	{
		qDebug() << "Invalid image. Size:" << imgSize.width << "x" << imgSize.height;
		return -1;
	}

	//截取图像边缘
	cv::Mat pt = img(cv::Rect(0, UP_CUT, imgSize.width, imgSize.height)); //pt = cropped image
	cv::imshow(WIN_NAME, pt);

	if (skip_pic == 120){  //跳过120张照片

		//if (click){
		cv::Mat canny;
		cv::Canny(pt, canny, 100, 100, 3);
		imshow("canny", canny);

		//Perform Warping
		cv::vector<cv::vector<cv::Point>> contours;    //储存轮廓
		cv::vector<cv::Vec4i> hierarchy;

		findContours(canny, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);    //获取轮廓
		cv::Mat linePic = cv::Mat::zeros(canny.rows, canny.cols, CV_8UC3);
		for (int index = 0; index < contours.size(); index++){
			drawContours(linePic, contours, index, cv::Scalar(rand() & 255, rand() & 255, rand() & 255), 1, 8/*, hierarchy*/);
		}

		cv::vector<cv::vector<cv::Point>> polyContours(contours.size());
		int maxArea = 0;
		for (int index = 0; index < contours.size(); index++){
			if (contourArea(contours[index]) > contourArea(contours[maxArea]))
				maxArea = index;
			approxPolyDP(contours[index], polyContours[index], 10, true);
		}

		cv::Mat polyPic = cv::Mat::zeros(pt.size(), CV_8UC3);
		drawContours(polyPic, polyContours, maxArea, cv::Scalar(0, 0, 255), 2);

		cv::vector<int>  hull;
		convexHull(polyContours[maxArea], hull, false);

		for (int i = 0; i < hull.size(); ++i){
			circle(polyPic, polyContours[maxArea][i], 10, cv::Scalar(rand() & 255, rand() & 255, rand() & 255), 3);
		}
		addWeighted(polyPic, 0.5, pt, 0.5, 0, pt);

		bool sorted = false;
		int n = 4;
		while (!sorted){
			for (int i = 1; i < n; i++){
				sorted = true;
				if (polyContours[maxArea][i - 1].x > polyContours[maxArea][i].x){
					std::swap(polyContours[maxArea][i - 1], polyContours[maxArea][i]);
					sorted = false;
				}
			}
			n--;
		}

		cv::Mat dst_warp, dst_warp2, dst_warpRotateScale, dst_warpTransformation, dst_warpFlip;
		dst_warp = pt(cv::Rect(0, UP_CUT, imgSize.width - 300, imgSize.height));
		cv::Point2f srcPoints[4], dstPoints[4];
		if (polyContours[maxArea][1].y < polyContours[maxArea][0].y){
			srcPoints[0] = polyContours[maxArea][1];
			srcPoints[2] = polyContours[maxArea][0];
		}
		else{
			srcPoints[0] = polyContours[maxArea][0];
			srcPoints[2] = polyContours[maxArea][1];
		}
		if (polyContours[maxArea][3].y < polyContours[maxArea][2].y){
			srcPoints[1] = polyContours[maxArea][3];
			srcPoints[3] = polyContours[maxArea][2];
		}
		else{
			srcPoints[1] = polyContours[maxArea][2];
			srcPoints[3] = polyContours[maxArea][3];
		}

		dstPoints[0] = cv::Point2f(0, 0);
		dstPoints[1] = cv::Point2f(canny.rows - 100, 0);  //bw.rows-200 -> shrink width
		dstPoints[2] = cv::Point2f(0, canny.cols - 200); //bw.cols-200 -> shrink height
		dstPoints[3] = cv::Point2f(canny.rows - 100, canny.cols - 200); //bw.rows-200 -> shrink width, bw.cols-200 -> shrink height

		cv::Mat M1 = cv::getPerspectiveTransform(srcPoints, dstPoints);//由四个点对计算透视变换矩阵  

		warpPerspective(pt, dst_warp, M1, dst_warp.size());//仿射变换  
		cv::imshow("After_Warp", dst_warp);

		cv::Mat crop[4][4];
		int k = 0, l = 0;
		for (int i = 0; i < 4; ++i){
			for (int j = 0; j < 4; ++j){
				if (j != 0)
				{
					k = 1;
				}
				else { k = 0; }
				if (i != 0)
				{
					l = 1;
				}
				else { l = 0; }
				crop[i][j] = dst_warp(cv::Rect(40 + 96 * (j)+k * 13 * j, 150 + 84 * (i)+l * 15 * i, 96, 84));

				cv::HOGDescriptor hog(cv::Size(32, 32), cv::Size(8, 8), cv::Size(4, 4), cv::Size(4, 4), 10);
				std::vector<float> descriptors;//HOG描述子向量
				hog.compute(crop[i][j], descriptors, cv::Size(4, 4));//计算HOG描述子
				cv::Mat M2 = cv::Mat(descriptors.size(), 1, CV_32FC1);
				memcpy(M2.data, descriptors.data(), descriptors.size()*sizeof(float));
				int r = svm.predict(M2);
				board[i][j] = r;
			}
		}
		std::cout << "\n"<<"\n";
		for (int k = 0; k < 4; ++k){
			for (int l = 0; l < 4; ++l){
				std::cout << '[' << board[k][l] << ']';
			}
			std::cout << std::endl;
		}
		//click = false;
		//对数组进行复制
		for (int k = 0; k < 4; ++k){
			for (int l = 0; l < 4; ++l){
				num.a[l][k] = board[l][k];
			}
		}
		//预处理
		tryleft(num, tmpl);
		tryright(num, tmpr);
		tryup(num, tmpu);
		trydown(num, tmpd);
		findmax(num);
		count(num);
		tryright(tmpu, tmp2ndu);
		tryright(tmpd, tmp2ndd);
		if (num.maxnum < 1024){
			if (left_done == 0 && !cntcmp(tmp2ndu, tmp2ndd) && cntcmp(tmp2ndd, tmpr) && ((max_in_corner(num) && max_in_corner(tmp2ndd)) || !max_in_corner(num)) && !compare(tmpd, tmp2ndd)){
				usrGameController::downmove();
			}
			else{
				if (left_done == 0 && cntcmp(tmp2ndu, tmpr) && ((max_in_corner(num) && max_in_corner(tmp2ndu)) || !max_in_corner(num)) && !compare(tmpu, tmp2ndu)){
					usrGameController::upmove();
				}
				else{
					if (compare(num, tmpr) && compare(num, tmpd) && compare(num, tmpu)){
						usrGameController::leftmove();
						left_done = 1;
					}
					else {
						if (compare(num, tmpr) && compare(num, tmpd)){
							usrGameController::upmove();
							up_done = 1;
						}
						else{
							if (compare(num, tmpr) && (!compare(num, tmpd))){
								usrGameController::downmove();
								up_done = 0;
							}
							else{
								if (up_done == 1 && left_done == 0 && !max_in_corner(num) && num.a[3][3] == 0 && num.a[num.max_row < 2 ? 2 : 3][3] == 0 && num.a[num.max_row < 1 ? 1 : 3][3] == 0){
									usrGameController::downmove();
									up_done = 0;
								}
								else {
									if (left_done == 1 && !max_in_corner(num) && (num.a[3][3] != 0 || num.a[3][num.max_col < 2 ? 2 : 3] != 0 || num.a[3][num.max_col < 1 ? 1 : 3] != 0)){
										usrGameController::upmove();
										up_done = 1;
									}
									else{
										usrGameController::rightmove();
										left_done = 0;
									}
								}
							}
						}
					}
				}
			}
		}
		else{
			tryup(tmpu, tmp3rdu);
			tryright(tmp3rdu, tmp3rdur);
			trydown(tmpd, tmp3rdd);
			tryright(tmp3rdd, tmp3rddr);
			if (left_done == 0 && !cntcmp(tmp3rdur, tmp3rddr) && cntcmp(tmp3rddr, tmpr)&&cntcmp(tmp3rddr,tmp2ndd)&& ((max_in_corner(num) && max_in_corner(tmp3rddr)) || !max_in_corner(num)) && !compare(tmp3rdd, tmp3rddr)){
				usrGameController::downmove();
			}
			else{
				if (left_done == 0 && cntcmp(tmp3rdur, tmpr)&&cntcmp(tmp3rdur,tmp2ndu) && ((max_in_corner(num) && max_in_corner(tmp3rdur)) || !max_in_corner(num)) && !compare(tmp3rdu, tmp3rdur)){
					usrGameController::upmove();
				}
				else{
					if (left_done == 0 && !cntcmp(tmp2ndu, tmp2ndd) && cntcmp(tmp2ndd, tmpr) && ((max_in_corner(num) && max_in_corner(tmp2ndd)) || !max_in_corner(num)) && !compare(tmpd, tmp2ndd)){
						usrGameController::downmove();
					}
					else{
						if (left_done == 0 && cntcmp(tmp2ndu, tmpr) && ((max_in_corner(num) && max_in_corner(tmp2ndu)) || !max_in_corner(num)) && !compare(tmpu, tmp2ndu)){
							usrGameController::upmove();
						}
						else{
							if (compare(num, tmpr) && compare(num, tmpd) && compare(num, tmpu)){
								usrGameController::leftmove();
								left_done = 1;
							}
							else {
								if (compare(num, tmpr) && compare(num, tmpd)){
									usrGameController::upmove();
									up_done = 1;
								}
								else{
									if (compare(num, tmpr) && (!compare(num, tmpd))){
										usrGameController::downmove();
										up_done = 0;
									}
									else{
										if (up_done == 1 && left_done == 0 && !max_in_corner(num) && num.a[3][3] == 0 && num.a[num.max_row < 2 ? 2 : 3][3] == 0 && num.a[num.max_row < 1 ? 1 : 3][3] == 0){
											usrGameController::downmove();
											up_done = 0;
										}
										else {
											if (left_done == 1 && !max_in_corner(num) && (num.a[3][3] != 0 || num.a[3][num.max_col < 2 ? 2 : 3] != 0 || num.a[3][num.max_col < 1 ? 1 : 3] != 0)){
												usrGameController::upmove();
												up_done = 1;
											}
											else{
												usrGameController::rightmove();
												left_done = 0;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		skip_pic = 0;
}

	return 0;
}

//鼠标回调函数
void mouseCallback(int event, int x, int y, int flags, void*param)
{
	usrGameController::MouseArgs* m_arg = (usrGameController::MouseArgs*)param;
	switch (event)
	{
	case CV_EVENT_MOUSEMOVE: // 鼠标移动时
	{
		if (m_arg->Drawing)
		{
			m_arg->box.width = x - m_arg->box.x;
			m_arg->box.height = y - m_arg->box.y;
		}
	}
		break;
	case CV_EVENT_LBUTTONDOWN:case CV_EVENT_RBUTTONDOWN: // 左/右键按下
	{
		m_arg->Hit = event == CV_EVENT_RBUTTONDOWN;
		m_arg->Drawing = true;
		m_arg->box = cvRect(x, y, 0, 0);
		//click = true;
	}
		break;
	case CV_EVENT_LBUTTONUP:case CV_EVENT_RBUTTONUP: // 左/右键弹起
	{
		m_arg->Hit = false;
		m_arg->Drawing = false;
		if (m_arg->box.width < 0)
		{
			m_arg->box.x += m_arg->box.width;
			m_arg->box.width *= -1;
		}
		if (m_arg->box.height < 0)
		{
			m_arg->box.y += m_arg->box.height;
			m_arg->box.height *= -1;
		}
	}
		break;
	}
}
#endif
