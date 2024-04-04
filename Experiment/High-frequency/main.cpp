#include "header.h"

/* High-frequency */
#include "task.h"

enum EHFType {
	Telea,
	pm,
	peeb,
	pepm
};

enum ETask {
	generateMask,
	inpainting,
	inpaintingAll,
	reconstruction,
	mergeimage,
};

enum EInpainting {
	recon,
	telea,
};

int main() {
	ETask Task = mergeimage;
	EInpainting Inpainting = recon;

	switch (Task) {
	case generateMask:
	{
		for (int i = 1; i <= 4; i++) {
			const std::string HolePath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/hole.png");
			const std::string GTPath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/gt.png");
			const std::string MaskPath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/mask.png");
			
			cv::Mat Mask;
			if (i == 2) {
				Mask = Task::generateMask(HolePath);
			}
			else {
				Mask = Task::generateMask(HolePath, GTPath);
			}
			cv::imwrite(MaskPath, Mask);
		}
		break;
	}
	case inpainting:
	{
		int i = 2;
		const std::string HolePath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/hole.png");
		const std::string GTPath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/gt.png");
		const std::string MaskPath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/mask.png");
		cv::Mat Hole = cv::imread(HolePath, 0);
		cv::Mat Mask = cv::imread(MaskPath, 0);
		cv::Mat GT = cv::imread(GTPath, 0);
		cv::Mat Result = Task::inpaintImage(Hole, Mask, GT);
		cv::imwrite("Images/Result.png", Result);
		break;
	}
	case reconstruction:
	{
		int i = 4;
		const std::string GTPath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/gt.png");
		const std::string MaskPath = HIGHFREQUENCY_DIR + std::string(std::to_string(i) + "/mask.png");
		cv::Mat Mask = cv::imread(MaskPath, 0);
		cv::Mat GT = cv::imread(GTPath, 0);
		cv::Mat Result = Task::reconstruct(GT, Mask);
		cv::imwrite("Images/reconstruct.png", Result);
		break;
	}
	case mergeimage:
	{
		const std::string ModelId = "4";
		cv::Mat Ours1 = cv::imread("MergeImages/"+ ModelId +"/1.png", 0);
		cv::Mat Ours2 = cv::imread("MergeImages/" + ModelId + "/2.png", 0);
		cv::Mat Mask = cv::imread("MergeImages/" + ModelId + "/mask.png", 0);
		cv::Mat Merge = Task::mergeImage(Ours1, Ours2, Mask);
		cv::imwrite("MergeImages/" + ModelId + "/Result.png", Merge);

		break;
	}
	case inpaintingAll:
	{
		const std::string ModelId = "1";
		const std::string GTPath = "Images/" + ModelId + "/gt.png";
		const std::string MaskPath = "Images/" + ModelId + "/mask.png";

		cv::Mat GT = cv::imread(GTPath, 0);
		cv::Mat Mask = cv::imread(MaskPath, 0);
		cv::Mat Res;

		switch (Inpainting) {
		case recon: {
			Res = Task::reconstruct(GT, Mask);
			cv::imwrite("Images/" + ModelId + "/recon.png", Res);
			break;
		}
		case telea:
			break;
		default:
			break;
		}
	}
	default:
		break;
	}


	return 0;
}