/**
  *	@brief midend example for lidar scans correction
 *	@date 2021
 *	@author Soso
 */

#include <stdio.h> // printf
#include "midend/Structures.h" // JacobiSVD

std::vector<CMeasurement2> LoadFrame(CScanCorrector &sc, int id, std::string path, Eigen::Matrix4d transform,
			std::string file = std::string(""))
{
	std::vector<CMeasurement2> m0 = sc.LoadMeasurements(id, 1, path, transform);
	std::vector<CMeasurement2> m1 = sc.LoadMeasurements(id, 2, path, transform);
	m0.insert(m0.end(), m1.begin(), m1.end());

	return m0;
}



/**
 *	@brief main
 *
 *	@param[in] n_arg_num is number of commandline arguments
 *	@param[in] p_arg_list is the list of commandline arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int main(int n_arg_num, const char **p_arg_list)
{
	CInputData data("/home/isolony/workspace/GEO/slam-midend-code/Data/calibration-imu-velodynes-center.txt", // imu -> velo center
					"/home/isolony/workspace/GEO/slam-midend-code/Data/calibration-velodynes.txt", // velo center -> velos
					"/mnt/ssd/isolony/dataset/GEO/new/druteva/druteva_imu.json", // IMU data
					"/mnt/ssd/isolony/dataset/GEO/new/druteva/frame.borders", // frame borders
					"/mnt/ssd/isolony/dataset/GEO/new/druteva/line-clouds/", // data to line clouds
					"/mnt/ssd/isolony/dataset/GEO/new/druteva/line-matches/", // data to frame matches
					"/mnt/ssd/isolony/dataset/GEO/new/druteva/inter-matches/", // data to inter matches
					"/mnt/ssd/isolony/dataset/GEO/new/druteva/line-clouds/", // data to lidar clouds
					"/home/isolony/workspace/GEO/slam-midend-code/outPCL/" // output dir
					);

	//CScanCorrector corrector(data, true);
	//corrector.CorrectRange( 26 /* frame from */,
	//						41 /* frame to */,
	//						10 /* n_internal frames */,
	//						30 /* export from */,
	//						35 /* export to */);

	CScanCorrector corrector(data, true);
	// 1. Create Scan Corrector

	corrector.AddFrame(LoadFrame(corrector, 0, std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/line-clouds/"), Eigen::Matrix4d::Identity()));
	// 2. Add Frame 0

	for(int a = 1; a < 3; ++a) {
	// LOOP START

		corrector.AddFrame(LoadFrame(corrector, a, std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/line-clouds/"), Eigen::Matrix4d::Identity()));
		// 3. Add Next Frame

		std::vector<std::pair<int, int> > matches;
		corrector.LoadMatches(a, a-1, std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/line-matches/"),
							  std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/inter-matches/"), matches, 0);
		corrector.AddFrameCorrespondences(matches);
		// 4. Add matches t-1 <> t // this initializes the 3D structure

		/*std::vector<std::pair<int, int> > imatches;
		corrector.LoadMatches(a, a, std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/inter-matches/"),
							  std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/inter-matches/"), imatches, 1);
		corrector.AddInterFrameCorrespondences(a, imatches);*/
		// 5. (optional) add internal matches

		if(a < 2)
			continue;
		/*std::vector<std::pair<int, int> > hmatches;
		corrector.LoadMatches(a, a-2, std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/line-matches/"),
									  std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/inter-matches/"), hmatches, 0);
		corrector.AddHistoryCorrespondence(a-2, a, hmatches);*/
		// 6. (optional) add history
	}
	// LOOP END

	corrector.OptimizeOffset();
	// 8. Optimize

	corrector.ExportData(0, std::string("/mnt/ssd/isolony/dataset/GEO/new/druteva/line-clouds/"),
							std::string("/home/isolony/workspace/GEO/slam-midend-code/outPCL/"));
	// 9. Export

	return 0;
}

/*
 *	end-of-file
 */
