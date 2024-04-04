#include "header.h"

/* Low-frequency */
#include "task.h"

enum ELFType {
	Plane,
	Quatric,
	Bezier_3,
	Bezier_11,
	Bezier_34,
	BSpline
};

enum ETaskType {
	Time,
	PCID,
	DGI,
	HeightMapSE,
	FittingMetric,
	generateHeightMap,
};

struct SFitPara {
	int d;
	int r;
	int it;
};

struct SRes {
	int work;
	int reco;
};

int main() {

	ELFType LowFre = Plane;
	ETaskType Task = generateHeightMap;
	SFitPara Para { 3, 5, 10 };
	SRes Res { 600, 600 };

	const std::string RawPath = LOWFREQUENCY_DIR + std::string("raw.ply");
	const std::string QuadricPath = LOWFREQUENCY_DIR + std::string("quadric.ply");
	const std::string BsplinePath = LOWFREQUENCY_DIR + std::string("bspline.ply");
	const std::string SavePath = "Result/result.ply";
	std::string LowPath = QuadricPath;

	switch (LowFre) {
	case Plane:
		break;
	case Quatric:
		LowPath = QuadricPath;
		break;
	case Bezier_3:
		LowPath = BsplinePath;
		Para = { 3, 0, 5 };
		break;
	case Bezier_11:
		LowPath = BsplinePath;
		Para = { 11, 0, 5 };
		break;
	case Bezier_34:
		LowPath = BsplinePath;
		Para = { 34, 0, 5 };
		break;
	case BSpline:
		LowPath = BsplinePath;
		break;
	default:
		break;
	}

	PC_t::Ptr pRaw = io::loadData(RawPath);
	PC_t::Ptr pLow = io::loadData(LowPath);
	PC_t::Ptr pOutput(new PC_t);

	switch (Task) {
	case Time:
		Para.it = 5;
		Task::calcFittingTime(pLow, Para.d, Para.r, Para.it);
		break;
	case PCID:
	{
		bool r = Task::runPCID(pRaw, pLow, pOutput, Res.work, Res.reco, Para.d, Para.r, Para.it);
		if (r) {
			pcl::io::savePLYFileBinary(SavePath, *pOutput);
		}
		break;
	}
	case DGI:
	{
		bool r = Task::runDGI(pRaw, pOutput, Res.work, Res.reco);
		if (r) {
			pcl::io::savePLYFileBinary(SavePath, *pOutput);
		}
		break;
	}
	case HeightMapSE:
	{
		if (LowFre == Plane) {
			core::PointCloudUtil::scale(pRaw, 0.01);
			bool r = Task::generateHeightMap(pRaw, Res.work, Res.reco);
		}
		else {
			core::PointCloudUtil::scale(pRaw, 0.01);
			core::PointCloudUtil::scale(pLow, 0.01);
			bool r = Task::generateHeightMap(pRaw, pLow, Res.work, Res.reco, Para.d, Para.r, Para.it);
		}
		break;
	}
	case FittingMetric:
	{

		break;
	}
	case generateHeightMap:
	{
		std::string ModelId = "4";
		const std::string LargePath = MAINEXPERIMENT_DIR + std::string("Large/" + ModelId + ".ply");
		const std::string HolePath = MAINEXPERIMENT_DIR + std::string("Hole/" + ModelId + ".ply");
		const std::string SubPath = MAINEXPERIMENT_DIR + std::string("Sub/" + ModelId + ".ply");
		PC_t::Ptr pLarge = io::loadData(LargePath);
		PC_t::Ptr pHole = io::loadData(HolePath);
		PC_t::Ptr pSub = io::loadData(SubPath);

		Task::generateHeightMap(pLarge, pHole, pSub, Res.work, Para.d, Para.r, Para.it, ModelId);
		break;
	}
	default:
		break;
	}

	return 0;
}