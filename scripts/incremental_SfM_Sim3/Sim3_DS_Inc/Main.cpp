#include "../UberLame_src/NewFix.h"
#include "../UberLame_src/CallStack.h"
#include "../UberLame_src/StlUtils.h"
#include "../UberLame_src/Integer.h"
#include "../UberLame_src/Tga.h"
#include <stdio.h>
#include <vector>
#include <string>
#include <map>

struct TCamVertex {
	size_t n_id;
	std::string s_initialization;

	inline bool operator ==(size_t _n_id) const
	{
		return n_id == _n_id;
	}
};

struct TInvDVertex {
	size_t n_id;
	size_t n_owner_cam_id;
	std::string s_initialization;

	inline bool operator ==(size_t _n_id) const
	{
		return n_id == _n_id;
	}
};

struct TEdgeProj {
	size_t n_landmark_id;
	std::vector<size_t> cam_id_chain; // if the length of the chain is 1, it is proj_self, otherwise it is proj_other
	std::string s_measurement;
};

std::string s_EatToken(std::string &r_s_line)
{
	_ASSERTE(!r_s_line.empty()); // the algorithms want a token out of this. not returning a token is not an option
	size_t n_token_end = r_s_line.find_first_of(" \t");
	std::string s_token;
	if(n_token_end == std::string::npos) { // no more spaces
		s_token.swap(r_s_line);
		return s_token;
	}
	s_token.insert(s_token.begin(), r_s_line.begin(), r_s_line.begin() + n_token_end);
	r_s_line.erase(0, n_token_end + 1);
	stl_ut::TrimSpace(r_s_line);
	_ASSERTE(!s_token.empty()); // the algorithms want a token out of this. not returning a token is not an option
	return s_token;
}

class CContiguousIds {
protected:
	size_t m_n_next_free_id;
	std::map<size_t, size_t> m_id_translation_table;
	// table for translating vertex ids

public:
	CContiguousIds()
		:m_n_next_free_id(size_t(-1))
	{}

	size_t operator ()(size_t n_old_id)
	{
		if(m_id_translation_table.count(n_old_id))
			return m_id_translation_table[n_old_id];
		return m_id_translation_table[n_old_id] = ++ m_n_next_free_id;
	}
};

void main()
{
	std::vector<TCamVertex> cameras;
	std::vector<TInvDVertex> landmarks;
	std::vector<TEdgeProj> edges;

	size_t n_graph_line_num = 0;
	//const char *p_s_input_file = "E:\\my-projects\\Robotics\\MonaschGit\\SLAM_plus_plus\\data\\newcollege_graph_new.txt";
	const char *p_s_input_file = "E:\\my-projects\\Robotics\\MonaschGit\\SLAM_plus_plus\\data\\vincent_fixed\\tum_frei3_graph.txt";
	//const char *p_s_input_file = "E:\\my-projects\\Robotics\\MonaschGit\\SLAM_plus_plus\\data\\vincent_gd\\simulation_graph.txt";
	FILE *p_fr = fopen(p_s_input_file, "r");
	while(!feof(p_fr)) {
		std::string s_line;
		if(!stl_ut::ReadLine(s_line, p_fr)) {
			fprintf(stderr, "fail\n");
			fclose(p_fr);
			return;
		}
		stl_ut::TrimSpace(s_line);
		if(s_line.empty())
			continue;
		// read lines from the file

		std::string s_token = s_EatToken(s_line);
		if(s_token == "VERTEX_CAM:SIM3") {
			TCamVertex v;
			v.n_id = atol(s_EatToken(s_line).c_str());
			_ASSERTE(!s_line.empty());
			v.s_initialization = s_line;
			cameras.push_back(v);
		} else if(s_token == "VERTEX:INVD") {
			TInvDVertex v;
			v.n_id = atol(s_EatToken(s_line).c_str());
			v.n_owner_cam_id = atol(s_EatToken(s_line).c_str());
			_ASSERTE(!s_line.empty());
			v.s_initialization = s_line;
			landmarks.push_back(v);
		} else if(s_token == "EDGE_PROJ_SELF") {
			TEdgeProj e;
			e.n_landmark_id = atol(s_EatToken(s_line).c_str());
			e.cam_id_chain.push_back(atol(s_EatToken(s_line).c_str()));
			_ASSERTE(!s_line.empty());
			e.s_measurement = s_line;
			edges.push_back(e);
		} else if(s_token == "EDGE_PROJ_OTHER") {
			TEdgeProj e;
			e.n_landmark_id = atol(s_EatToken(s_line).c_str());
			size_t n_chain_length = atol(s_EatToken(s_line).c_str());
			_ASSERTE(n_chain_length > 1); // otherwise it would be self
			for(size_t j = 0; j < n_chain_length; ++ j)
				e.cam_id_chain.push_back(atol(s_EatToken(s_line).c_str())); // eat the camera chain
			_ASSERTE(!s_line.empty());
			e.s_measurement = s_line; // measurement
			edges.push_back(e);
		} else {
			fprintf(stderr, "fail\n");
			fclose(p_fr);
			return;
		}
		++ n_graph_line_num;
	}
	fclose(p_fr);
	// parse the graph file

	std::vector<size_t> camera_order;
	for(size_t i = 0, n = cameras.size(); i < n; ++ i)
		camera_order.push_back(cameras[i].n_id);
	// we assume, unlike the bash script, that the camera order will not change

	//std::map<std::pair<size_t, size_t>, TEdgeProj*> edge_map; // unused
	std::map<size_t, std::vector<TEdgeProj*> > camera_edge_map;
	std::map<size_t, std::vector<TEdgeProj*> > ladnmark_edge_map;
	for(size_t i = 0, n = edges.size(); i < n; ++ i) {
		//_ASSERTE(!edge_map.count(std::make_pair(edges[i].n_landmark_id, edges[i].cam_id_chain.front()))); // make sure this is not there are no double observations
		//edge_map[std::make_pair(edges[i].n_landmark_id, edges[i].cam_id_chain.front())] = &edges[i]; // indexed by (landmark, observing camera)
		camera_edge_map[edges[i].cam_id_chain.front()].push_back(&edges[i]); // indexed by observing camera
		ladnmark_edge_map[edges[i].n_landmark_id].push_back(&edges[i]); // indexed by landmark
	}
	// make a map of edges to speed up the lookup

	std::map<size_t, size_t> landmark_obs; // indexed by landmark id
	// get a list of landmark observation numbers

	CContiguousIds id_translate;

	size_t n_printed_vertex_num = 0;
	size_t n_last_printed_edge_num = 0;
	std::vector<std::pair<int, int> > printed_edges;

	for(size_t i = 0, n = camera_order.size(); i < n; ++ i) {
		size_t n_camera_id = camera_order[i];
		const TCamVertex &t_camera = *std::find(cameras.begin(), cameras.end(), camera_order[i]); // small array, not worth making a reverse lookup for this
		_ASSERTE(n_camera_id == t_camera.n_id);
		// get the next camera in order

		printf("VERTEX_CAM:SIM3 " PRIsize " %s\n", id_translate(n_camera_id), t_camera.s_initialization.c_str());
		-- n_graph_line_num;
		++ n_printed_vertex_num;
		// initialize camera vertex

		std::vector<size_t> added_landmarks;

		const std::vector<TEdgeProj*> &r_edges = camera_edge_map[n_camera_id];
		for(size_t j = 0, m = r_edges.size(); j < m; ++ j) {
			size_t n_landmark_id = r_edges[j]->n_landmark_id;
			if(++ landmark_obs[n_landmark_id] == 2) { // the landmark was observed twice
				const TInvDVertex &t_landmark = *std::find(landmarks.begin(), landmarks.end(), n_landmark_id); // the search only performed landmarks.size() times
				// get a landmark that is being initialized

				if(std::find(camera_order.begin(), camera_order.begin() + (i + 1), t_landmark.n_owner_cam_id) == camera_order.begin() + (i + 1)) {
					landmark_obs[n_landmark_id] = 1; // hack - the landmark is seen twice, yet the camera that owns it is not in the graph yet. decrease the observation count and let the next camera try inseting it again, until the owner is in the graph.
					continue;
				}
				// make sure that the owner camera was already introduced

				printf("VERTEX:INVD " PRIsize " " PRIsize " %s\n", id_translate(n_landmark_id),
					id_translate(t_landmark.n_owner_cam_id), t_landmark.s_initialization.c_str());
				-- n_graph_line_num;
				++ n_printed_vertex_num;
				// initialize the landmark vertex

				added_landmarks.push_back(n_landmark_id);
				// remember it for later
			}
		}
		// see which landmarks were observed and introduce the ones that are being observed for the second time

		for(size_t j = 0, m = r_edges.size(); j < m; ++ j) {
			const TEdgeProj &r_edge = *r_edges[j];
			size_t n_landmark_id = r_edge.n_landmark_id;
			if(landmark_obs[n_landmark_id] >= 2) { // if the landmarks was seen by at least two by now ...
				if(r_edge.cam_id_chain.size() == 1) {
					printf("EDGE_PROJ_SELF " PRIsize " " PRIsize " %s\n", id_translate(n_landmark_id),
						id_translate(r_edge.cam_id_chain.front()), r_edge.s_measurement.c_str());
				} else {
					printf("EDGE_PROJ_OTHER " PRIsize " " PRIsize, id_translate(n_landmark_id), r_edge.cam_id_chain.size());
					for(size_t k = 0, o = r_edge.cam_id_chain.size(); k < o; ++ k)
						printf(" " PRIsize, id_translate(r_edge.cam_id_chain[k]));
					printf(" %s\n", r_edge.s_measurement.c_str());
				}
				printed_edges.push_back(std::make_pair(n_landmark_id, r_edge.cam_id_chain.front()));
				-- n_graph_line_num;
			}
		}
		// print camera edges

		for(size_t j = 0, m = added_landmarks.size(); j < m; ++ j) {
			size_t n_landmark_id = added_landmarks[j];
			const std::vector<TEdgeProj*> &r_edges = ladnmark_edge_map[n_landmark_id];
			for(size_t k = 0, o = r_edges.size(); k < o; ++ k) {
				const TEdgeProj &r_edge = *r_edges[k];
				size_t n_camera_id = r_edge.cam_id_chain.front(); // the observing camera
				if(std::find(camera_order.begin(), camera_order.begin() + i, n_camera_id) != camera_order.begin() + i) { // exclude camera i
					if(r_edge.cam_id_chain.size() == 1) {
						printf("EDGE_PROJ_SELF " PRIsize " " PRIsize " %s\n", id_translate(n_landmark_id),
							id_translate(r_edge.cam_id_chain.front()), r_edge.s_measurement.c_str());
					} else {
						printf("EDGE_PROJ_OTHER " PRIsize " " PRIsize, id_translate(n_landmark_id), r_edge.cam_id_chain.size());
						for(size_t k = 0, o = r_edge.cam_id_chain.size(); k < o; ++ k)
							printf(" " PRIsize, id_translate(r_edge.cam_id_chain[k]));
						printf(" %s\n", r_edge.s_measurement.c_str());
					}
					printed_edges.push_back(std::make_pair(n_landmark_id, r_edge.cam_id_chain.front()));
					-- n_graph_line_num;
					// if this is an edge to a *strictly* previous camera, it means that we have not printed this edge yet
				}
			}
		}
		// print edges of the other cameras

		if(i) {
			/*{
				size_t n_res = std::min(size_t(640), n_printed_vertex_num);
				TBmp *p_bitmap = TBmp::p_Alloc(n_res, n_res);
				if(p_bitmap) {
					double f_scale = double(n_res - 1) / n_printed_vertex_num;
					uint32_t n_alpha = max(10, int(255 * f_scale + .5)) << 24;
					p_bitmap->Clear(-1); // white

					p_bitmap->DrawLine_SP(0, 0, n_res, n_res, 0xff0000ff);
					// diagonal

					for(size_t j = 0, m = printed_edges.size(); j < m; ++ j) {
						float a = printed_edges[j].first * f_scale, b = printed_edges[j].second * f_scale;
						uint32_t n_color = ((j < n_last_printed_edge_num)? 0xff0000ff : 0xffff0000) | n_alpha; // alpha unused, for large datasets causes "bleaching", due to extreme point overdraw
						p_bitmap->PutPixel_AA(a, a, n_color);
						p_bitmap->PutPixel_AA(a, b, n_color);
						p_bitmap->PutPixel_AA(b, a, n_color);
						p_bitmap->PutPixel_AA(b, b, n_color);
					}
					// off-diagonal

					char p_s_filename[256];
					stl_ut::Format(p_s_filename, sizeof(p_s_filename), "graph_%04" _PRIsize ".tga", i);
					CTgaCodec::Save_TGA(p_s_filename, *p_bitmap, false, true);

					p_bitmap->Delete();
				}

				n_last_printed_edge_num = printed_edges.size();
			}*/
			// t_odo - create the bitmaps here

			printf("CONSISTENCY_MARKER\n");
		}
		// added a camera, landmarks and all the edges, the graph is consistent
	}

	for(size_t i = 0, n = landmarks.size(); i < n; ++ i) {
		if(landmark_obs[landmarks[i].n_id] < 2)
			fprintf(stderr, "error: landmark " PRIsize " did not have two observations and was ommited\n", landmarks[i].n_id);
	}

	if(n_graph_line_num != 0)
		fprintf(stderr, "error: we did not print something / printed something twice\n");

	fprintf(stderr, "done\n");

}
