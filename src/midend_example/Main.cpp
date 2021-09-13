/**
  *	@brief midend example for lidar scans correction
 *	@date 2021
 *	@author Soso
 */

#include <stdio.h> // printf
#include "midend/Structures.h" // JacobiSVD

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

	CScanCorrector corrector(data, true);
	corrector.CorrectRange( 26 /* frame from */,
							41 /* frame to */,
							10 /* n_internal frames */,
							30 /* export from */,
							35 /* export to */);

	return 0;
}

/*
 *	end-of-file
 */
