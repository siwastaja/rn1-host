/*
	This module connects to a SoftKinetic DS325 camera, processes the depthmap and forms an object map.

	Object map values defined in tof3d.h.

	The camera is expected to be mounted on the top of the robot, looking downwards, slightly tilted to
	show the front area of the robot.

	DS325 cannot be used as a forward-looking camera since it lacks dealiasing functionality, so that
	longer distances than the measurement range are aliased as short distances. (For a downward-looking
	camera, it's acceptable that a long distance (a huge pit!) is aliased as a short distance (so it
	appears like a wall or a tabletop), as it will be avoided anyway.

	Once our own 3D TOF camera (with powerful light source to provide long range, + dealiasing)
	is developed, the DS325 and lidar will be both replaced with a few of those.

	This module is in C++ because the DS325 SDK is in C++.
*/

#include "DepthSense.hxx"
#include <list>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include "tof3d.h"

#include "datatypes.h" // for tof3d_scan_t

class Softkinetic_tof
{	

public:
	
	struct NodeInfo
	{
		DepthSense::Node node;
	};
	
	Softkinetic_tof();
	virtual ~Softkinetic_tof();
	
	void init(bool calibrate);
	void release();
	void run();

	void do_quit();
	
	void onDeviceAdded(DepthSense::Context context, DepthSense::Device device);
	void onDeviceRemoved(DepthSense::Context context, DepthSense::Device device);

	void onNodeAdded(DepthSense::Device device, DepthSense::Node node);
	void onNodeRemoved(DepthSense::Device device, DepthSense::Node node);

	void onNewDepthNodeSampleReceived(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);
private:
	DepthSense::Context _context;
	std::list<DepthSense::Device> _devices;
	std::list<NodeInfo*> _nodeInfos;
	bool _initialized;
	bool _streaming;
	bool _error;
	bool _calibrating;
	bool _mode30;
	bool _temporal_smooth;
	bool _dbg_print;
};


using namespace std;
using namespace DepthSense;
using namespace DepthSense::Utils;

#define CONTEXT_QUIT(context)						   \
do													  \
{													   \
	try {											   \
		context.quit();								 \
	}												   \
	catch (DepthSense::InvalidOperationException&)	  \
	{												   \
	}												   \
} while (0)


Softkinetic_tof::Softkinetic_tof()
: _initialized(false)
, _streaming(false)
, _error(false)
, _calibrating(false)
, _mode30(false)
, _temporal_smooth(false)
, _dbg_print(false)
{
}

Softkinetic_tof::~Softkinetic_tof()
{
	release();
}

void Softkinetic_tof::init(bool calibrate)
{
	if (_initialized)
	{
		return;
	}

	if(calibrate)
	{
		printf("  TOF3D MODULE Running floor calibration. Move the robot carefully around while ensuring that only level floor is seen. Calibration ends automatically after 500 frames.\n");
	}
	_calibrating = calibrate;

	_context = Context::createStandalone();

	_context.deviceAddedEvent().connect(this, &Softkinetic_tof::onDeviceAdded);
	_context.deviceRemovedEvent().connect(this, &Softkinetic_tof::onDeviceRemoved);
	std::vector<Device> devices = _context.getDevices();
	for (unsigned int i = 0; i < devices.size(); i++)
	{
		onDeviceAdded(_context, devices[i]);
		std::vector<Node> nodes = devices[i].getNodes();
		for (unsigned int j = 0; j < nodes.size(); j++)
		{
			onNodeAdded(devices[i], nodes[j]);
		}
	}
	_initialized = true;
}


void Softkinetic_tof::release()
{
	if (!_initialized)
	{
		return;
	}
	std::vector<Node> registeredNodes = _context.getRegisteredNodes();
	
	for (unsigned int i = 0; i < registeredNodes.size(); i++)
	{
		if (registeredNodes[i].is<DepthNode>())
		{
			DepthNode n = registeredNodes[i].as<DepthNode>();
			n.newSampleReceivedEvent().disconnect
				(this, &Softkinetic_tof::onNewDepthNodeSampleReceived);
		}
	}
	
	std::list<Device>::iterator it;

	if (_streaming)
	{
		_context.stopNodes();
	}

	for (it = _devices.begin(); it != _devices.end(); it++)
	{
		Device device = *it;
		device.nodeRemovedEvent().disconnect(this, &Softkinetic_tof::onNodeRemoved);
		device.nodeAddedEvent().disconnect(this, &Softkinetic_tof::onNodeAdded);
	}
	
	_context.deviceRemovedEvent().disconnect(this, &Softkinetic_tof::onDeviceRemoved);
	_context.deviceAddedEvent().disconnect(this, &Softkinetic_tof::onDeviceAdded);

	CONTEXT_QUIT(_context);
	
	while (!_nodeInfos.empty())
	{
		NodeInfo* info = _nodeInfos.front();
		_nodeInfos.pop_front();
		delete info;
	}

	_devices.clear();
	_initialized = false;
}


void Softkinetic_tof::run()
{
	if (!_error)
	{
		_context.run();
	}
	release();
}

void Softkinetic_tof::onDeviceAdded(Context context, Device device)
{
	if (_error) return;

	DepthSense::Device::Model model = device.getModel();

	printf("  TOF3D MODULE Found device model %s with serial %s\n", DepthSense::Device::Model_toString(model).c_str(),
		device.getSerialNumber().c_str());
	device.nodeAddedEvent().connect(this, &Softkinetic_tof::onNodeAdded);
	device.nodeRemovedEvent().connect(this, &Softkinetic_tof::onNodeRemoved);
	
	_devices.push_back(device);
}


void Softkinetic_tof::onDeviceRemoved(Context context, Device device)
{
	printf("  TOF3D MODULE ERROR: Device removed\n");
	_devices.remove(device);
}

void Softkinetic_tof::onNodeAdded(Device device, Node node)
{
	if (_error) return;

	int32_t width = 0;
	int32_t height = 0;
	
	if (node.is<DepthNode>())
	{
		DepthNode n = node.as<DepthNode>(); 

		DepthNode::Configuration configuration = n.getConfiguration();

			std::vector<DepthNode::Configuration> configurations = n.getConfigurations();
			bool doSetConfiguration = false;
			for (unsigned int i = 0; i < configurations.size(); i++)
			{
				#ifdef PULU1
				if(configurations[i].framerate == 25 && strcmp(DepthNode::CameraMode_toString(configurations[i].mode).c_str(), "CloseMode") == 0)
				#else
				if(configurations[i].framerate == (_mode30?30:6) && strcmp(DepthNode::CameraMode_toString(configurations[i].mode).c_str(), "LongRange") == 0)
				#endif
				{
					printf("  TOF3D MODULE  Found requested config\n");
					configuration = configurations[i];
					doSetConfiguration = true;
				}

			}

			if(!doSetConfiguration)
			{
				printf("  TOF3D MODULE WARN:  Requested config not found; using the default. This may not work with the floor calibration.\n");
			}

			n.setEnableDepthMap(true);
			n.setEnableConfidenceMap(true);
			if (doSetConfiguration)
			{

				try {
					_context.requestControl(n, 0);
				}

				catch (Exception&)
				{
					printf("  TOF3D MODULE ERROR: Control request failed.\n");
					_error = true;
					CONTEXT_QUIT(_context);
					return;
				}

				try {
					n.setConfiguration(configuration);
					_context.releaseControl(n);
				}
				catch (std::exception&)
				{
					printf("  TOF3D MODULE ERROR: Configuration setup failed.\n");

					_error = true;
					CONTEXT_QUIT(_context);
					return;
				}
			}

			n.newSampleReceivedEvent().connect
				(this, &Softkinetic_tof::onNewDepthNodeSampleReceived);

		DepthSense::FrameFormat_toResolution(configuration.frameFormat, &width, &height);
	
		try
		{
			_context.registerNode(node);
		}
		catch(Exception& e)
		{
			printf("  TOF3D MODULE ERROR: Node registration failed.\n");
			_error = true;
			CONTEXT_QUIT(_context);
			return;
		}

		try
		{
			_context.startNodes();
			_streaming = true;
		}
		catch(Exception& e)
		{
			printf("  TOF3D MODULE ERROR: Streaming startup failed.\n");
			_error = true;
			CONTEXT_QUIT(_context);
			return;
		}
		if (!node.is<UnsupportedNode>())
		{
			NodeInfo* info = new NodeInfo;
		
			info->node = node;
		
			_nodeInfos.push_back(info);
		}
	}
}


void Softkinetic_tof::onNodeRemoved(Device device, Node node)
{
	printf("  TOF3D MODULE ERROR: Node removed\n");
	std::list<NodeInfo*>::iterator it;
	for (it = _nodeInfos.begin(); it != _nodeInfos.end(); it++)
	{
		if ((*it)->node == node)
		{
			NodeInfo* info = *it;
			_nodeInfos.remove(info);
			delete info;
			break;
		}
	}
}

void Softkinetic_tof::do_quit()
{
	CONTEXT_QUIT(_context);
}


#define HMAP_TEMPO 3

static bool quit = false;

static int16_t hmap_calib[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];

tof3d_scan_t tof3ds[TOF3D_RING_BUF_LEN];
int tof3d_wr;
int tof3d_rd;

extern "C" tof3d_scan_t* get_tof3d(void);

tof3d_scan_t* get_tof3d(void)
{
	if(tof3d_wr == tof3d_rd)
	{
		return 0;
	}
	
	tof3d_scan_t* ret = &tof3ds[tof3d_rd];
	tof3d_rd++; if(tof3d_rd >= TOF3D_RING_BUF_LEN) tof3d_rd = 0;
	return ret;
}

extern pthread_mutex_t cur_pos_mutex;
extern int32_t cur_ang, cur_x, cur_y;

int32_t tof3d_obstacle_levels[3];

void Softkinetic_tof::onNewDepthNodeSampleReceived(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
	if(quit)
	{
		CONTEXT_QUIT(_context);
		return;
	}

	#ifdef PULU1
	//Drop every other frame: 25 FPS -> 12.5 FPS
	static int drop_cnt = 0;
	drop_cnt++;
	if(drop_cnt&1) return;
	#else

	#endif

	extern int32_t cur_pos_invalid_for_3dtof;
	int32_t is_invalid;
	pthread_mutex_lock(&cur_pos_mutex);
	is_invalid = cur_pos_invalid_for_3dtof;
	pthread_mutex_unlock(&cur_pos_mutex);

	if(is_invalid)
		return;

	if(!_calibrating)
	{
		int next = tof3d_wr+1; if(next >= TOF3D_RING_BUF_LEN) next = 0;
		static int ignore_print_cnt = 0;
		if(next == tof3d_rd)
		{
			if(ignore_print_cnt == 0)
				printf("  TOF3D MODULE WARNING: tof3d ring buffer overrun prevented by ignoring a scan.\n");
			else if(ignore_print_cnt%20 == 0)
				printf("  TOF3D MODULE WARNING: still ignoring scans\n");

			ignore_print_cnt++;
			return;
		}
		else
			ignore_print_cnt = 0;
	}

	const int16_t* depthMap = (const int16_t*) data.depthMap;
	const int16_t* confiMap = (const int16_t*) data.confidenceMap;

	if (!depthMap || !confiMap)
	{
		printf("  TOF3D MODULE ERROR: depthMap or confiMap NULL\n");
		return;
	}

	pthread_mutex_lock(&cur_pos_mutex);
	tof3ds[tof3d_wr].robot_pos.ang = cur_ang;
	tof3ds[tof3d_wr].robot_pos.x = cur_x;
	tof3ds[tof3d_wr].robot_pos.y = cur_y;
	pthread_mutex_unlock(&cur_pos_mutex);

	int32_t hmap_accum[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
	int16_t hmap_nsamples[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
	int16_t hmap[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
	static int32_t hmap_temporal_acc[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
	static int hmap_temporal_idx = 0;
	static int32_t hmap_calib_accum[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
	static int16_t hmap_calib_cnt[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
	int16_t hmap_avgd[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
	for(int xx=0; xx < TOF3D_HMAP_XSPOTS; xx++) { for(int yy=0; yy < TOF3D_HMAP_YSPOTS; yy++)
		{hmap_accum[xx][yy] = 0; hmap_nsamples[xx][yy] = 0; hmap_avgd[xx][yy] = 0;} }

	const float ang_per_pixel = (72.44/*deg horizontal fov*/ /320.0)/360.0*2.0*M_PI;
	const float top_cam_ang = 
		#ifdef PULU1
		(39.0)/360.0*2.0*M_PI;
		#else
		(32.0)/360.0*2.0*M_PI;
		#endif

	int ignored = 0;
	int confi_ignored = 0;

	for(int py=2; py < 240-2; py+=2)
	{
		for(int px=0+2; px < 320-2; px+=2)
		{
			#ifdef PULU1
			float pxang  = (float)(px-160) * ang_per_pixel;
			float pyang  = (float)(120-py) * ang_per_pixel;
			#else
			float pxang  = (float)(160-px) * ang_per_pixel;
			float pyang  = (float)(py-120) * ang_per_pixel;
			#endif
			float d = 0;

			if(confiMap[py*320+px] < (_calibrating?(_mode30?75:120):(_mode30?150:240)))
			{
				confi_ignored++;
				continue;
			}

			float biggest = -9999.9;
			float smallest = 9999.9;
			for(int ix=-2; ix<=2; ix++)
			{
				for(int iy=-2; iy<=2; iy++)
				{
					float dnow = depthMap[(py+iy)*320+(px+ix)];
					if(dnow > biggest) {biggest = dnow;}
					if(dnow < smallest) {smallest = dnow;}
					d += dnow;
				}
			}

			d = (d - biggest - smallest)/23.0;


//extern int cal_x_d_offset; //= 0;
//extern int cal_y_d_offset; //= 0;
//extern float cal_x_offset; //= 40.0;
//extern float cal_y_offset; //= 0;
//extern float cal_x_sin_mult; //= 1.125;
//extern float cal_y_sin_mult; //= 1.125;

			#ifdef PULU1
				if(d < 0 || d > 1000) continue;
			#else
				if(d < 0 || d > 1800) continue;
			#endif

//			float x = ((d+cal_x_d_offset) * (1.0/cos(pyang)) * sin(pyang+top_cam_ang))*cal_x_sin_mult + cal_x_offset;
//			float y = ((d+cal_y_d_offset) * (1.0/cos(pxang)) * sin(pxang))*cal_y_sin_mult + cal_y_offset;
//			float z = -1.0 * d * (1.0/cos(pyang)) * (1.0/cos(pxang)) * cos(pyang+top_cam_ang) + 900.0;

			float x = ((d) * (1.0/cos(pyang)) * sin(pyang+top_cam_ang))*1.15;
			float y = ((d) * (1.0/cos(pxang)) * sin(pxang))*1.15
				#ifdef PULU1
					-20.0;
				#else
					;
				#endif
			float z = -1.0 * d * (1.0/cos(pyang)) * (1.0/cos(pxang)) * cos(pyang+top_cam_ang) + 
				#ifdef PULU1
					280.0;
				#else
					900.0;
				#endif


			int xspot = (int)(x / (float)TOF3D_HMAP_SPOT_SIZE);
			int yspot = (int)(y / (float)TOF3D_HMAP_SPOT_SIZE) + TOF3D_HMAP_YMIDDLE;

			if(xspot < 0 || xspot >= TOF3D_HMAP_XSPOTS || yspot < 0 || yspot >= TOF3D_HMAP_YSPOTS)
			{
				ignored++;
				continue;
			}

			//if(z > hmap_accum[xspot][yspot]) hmap_accum[xspot][yspot] = z;

			hmap_accum[xspot][yspot] += z;

			hmap_nsamples[xspot][yspot]++;
		}
	}

	for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
	{
		for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
		{
			if(hmap_nsamples[sx][sy] > 10)
			{
				hmap[sx][sy] = hmap_accum[sx][sy]/hmap_nsamples[sx][sy];
				if(_calibrating)
				{
					hmap_calib_accum[sx][sy] += hmap[sx][sy];
					hmap_calib_cnt[sx][sy]++;
				}
				else
				{
					if(hmap_calib[sx][sy] == -9999)
						hmap[sx][sy] = -9999;
					else
						hmap[sx][sy] -= hmap_calib[sx][sy];
				}

			}
			else
				hmap[sx][sy] = -9999;

		}
	}

	#define NUM_CALIB_CNT 500
	if(_calibrating)
	{
		static int calib_cnt = 0;
		calib_cnt++;
		if(calib_cnt > NUM_CALIB_CNT)
		{
			FILE* floor = fopen("tof_zcalib.raw", "w");

			int32_t avg_accum = 0;
			int avg_cnt = 0;

			for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
			{
				for(int sx = 0; sx < TOF3D_HMAP_YSPOTS; sx++)
				{
					if(hmap_calib_cnt[sx][sy] < 10)
						hmap_calib[sx][sy] = -9999;
					else
					{
						hmap_calib[sx][sy] = hmap_calib_accum[sx][sy] / hmap_calib_cnt[sx][sy];
						if(hmap_calib_cnt[sx][sy] < 50)
							hmap_calib[sx][sy] /= 2; // to avoid overcorrecting in tricky cases

						avg_accum += hmap_calib[sx][sy];
						avg_cnt++;

					}
				}
			}

			int32_t avg = avg_accum / avg_cnt;
			int body_ignores = 0;

			int16_t hmap_calib_copy[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];

			memcpy(hmap_calib_copy, hmap_calib, sizeof(hmap_calib));

			// Generate robot body ignores:
			for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
			{
				for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
				{
					if(hmap_calib_copy[sx][sy] > avg + 70 || hmap_calib_copy[sx][sy] == -9999)
					{
						body_ignores++;
						hmap_calib[sx+0][sy+0] = -9999;
						if(sy>0) hmap_calib[sx+0][sy-1] = -9999;
						if(sy<TOF3D_HMAP_YSPOTS-1) hmap_calib[sx+0][sy+1] = -9999;
						if(sx < TOF3D_HMAP_XSPOTS-1)
						{
							hmap_calib[sx+1][sy+0] = -9999;
							if(sy>0) hmap_calib[sx+1][sy-1] = -9999;
							if(sy<TOF3D_HMAP_YSPOTS-1) hmap_calib[sx+1][sy+1] = -9999;
						}
						if(sx > 0)
						{
							hmap_calib[sx-1][sy+0] = -9999;
							if(sy>0) hmap_calib[sx-1][sy-1] = -9999;
							if(sy<TOF3D_HMAP_YSPOTS-1) hmap_calib[sx-1][sy+1] = -9999;
						}
					}
				}
			}
			fwrite(hmap_calib, 2, TOF3D_HMAP_XSPOTS*TOF3D_HMAP_YSPOTS, floor);
			fclose(floor);

			for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
			{
				for(int sx = 0; sx < TOF3D_HMAP_YSPOTS; sx++)
				{
					if(hmap_calib[sx][sy] == -9999)
						printf(" --  ");
					else
						printf("%4d ", hmap_calib[sx][sy]);
				}
				printf("\n");
			}

			printf("------------------------------------------------------------------------\n");
			printf("------------------------- CALIBRATION FINISHED -------------------------\n");
			printf("------------------------------------------------------------------------\n");
			printf(" %d frames samples    %d spots used (out of %d)\n", NUM_CALIB_CNT, avg_cnt, TOF3D_HMAP_XSPOTS*TOF3D_HMAP_YSPOTS);
			printf(" avg corr = %d     ignored %d locations (+ their neighbors) as robot body\n", avg, body_ignores);
			printf("------------------------------------------------------------------------\n");
			CONTEXT_QUIT(_context);
		}
		return;
	}

	if(_temporal_smooth)
	{
		for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
		{
			for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
			{
				hmap_temporal_acc[sx][sy] += hmap[sx][sy];
			}
		}
		hmap_temporal_idx++;

		if(hmap_temporal_idx >= HMAP_TEMPO)
		{
			for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
			{
				for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
				{
					hmap[sx][sy] = hmap_temporal_acc[sx][sy] / hmap_temporal_idx;
					hmap_temporal_acc[sx][sy] = 0;
				}
			}
			hmap_temporal_idx = 0;
		}
		else
		{
			return;
		}
	}

	// ------------ GENERATE SOFT AVGD MAP ---------

//	for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
//		for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
//			hmap_avgd[sx][sy] = 0;

	for(int sx = 3; sx < TOF3D_HMAP_XSPOTS-3; sx++)
	{
		for(int sy = 3; sy < TOF3D_HMAP_YSPOTS-3; sy++)
		{
			int nsamp = 0;
			int acc = 0;

			for(int ix=-6; ix<=6; ix++)
			{
				for(int iy=-6; iy<=6; iy++)
				{
					if(sx+ix < 0 || sx+ix > TOF3D_HMAP_XSPOTS-1 || sy+iy < 0 || sy+iy > TOF3D_HMAP_YSPOTS-1)
						continue;

					if(hmap[sx+ix][sy+iy] > -50 && hmap[sx+ix][sy+iy] < 50)
					{
						nsamp++;
						acc += hmap[sx+ix][sy+iy];
					}
				}
			}

			if(nsamp < 40 || hmap[sx][sy] < -1000)
				hmap_avgd[sx][sy] = 0;
			else
				hmap_avgd[sx][sy] = hmap[sx][sy] - acc/nsamp;
		}
	}

	// ------------ GENERATE OBJMAP BASED ON HMAP and AVGD -------------


	int obst_cnts[3] = {0,0,0}; // [0] = very far counts, [1] somewhat near, [2] very near.

//	for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
//		for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
//			tof3ds[tof3d_wr].objmap[sy*TOF3D_HMAP_XSPOTS+sx] = hmap[sx][sy]<-1000?-100:(hmap[sx][sy]/20);



	for(int sx = 1; sx < TOF3D_HMAP_XSPOTS-1; sx++)
	{
		for(int sy = 1; sy < TOF3D_HMAP_YSPOTS-1; sy++)
		{
			int nearness = 0;
			if(sx < (500)/TOF3D_HMAP_SPOT_SIZE &&
			   sy > TOF3D_HMAP_YMIDDLE - (280)/TOF3D_HMAP_SPOT_SIZE &&
			   sy < TOF3D_HMAP_YMIDDLE + (280)/TOF3D_HMAP_SPOT_SIZE)
				nearness = 2;
			else if(sx < (1000)/TOF3D_HMAP_SPOT_SIZE &&
			   sy > TOF3D_HMAP_YMIDDLE - (360)/TOF3D_HMAP_SPOT_SIZE &&
			   sy < TOF3D_HMAP_YMIDDLE + (360)/TOF3D_HMAP_SPOT_SIZE)
				nearness = 1;


			int8_t val = TOF3D_SEEN;
			if(hmap[sx][sy] < -998 || hmap_avgd[sx][sy] < -998)
				val = TOF3D_UNSEEN;
			else if(hmap[sx][sy] < -200 || hmap_avgd[sx][sy] < -110)
			{
				val = TOF3D_DROP;

				obst_cnts[nearness] += 8;
			}
			else if(hmap[sx][sy] < -170 || hmap_avgd[sx][sy] < -80)
			{
				val = TOF3D_POSSIBLE_DROP;
				obst_cnts[nearness] += 1;
			}
			else if(hmap[sx][sy] > 200 || hmap_avgd[sx][sy] > 120)
			{
				val = TOF3D_WALL;
				obst_cnts[nearness] += 8;
			}
			else if(hmap[sx][sy] > 100 || hmap_avgd[sx][sy] > 40)
			{
				val = TOF3D_BIG_ITEM;
				obst_cnts[nearness] += 8;
			}
			else if(hmap[sx][sy] > 90 || hmap_avgd[sx][sy] > 25)
			{
				val = TOF3D_SMALL_ITEM;
				obst_cnts[nearness] += 4;
			}
			else if(hmap[sx][sy] > 80 || hmap_avgd[sx][sy] > 15)
			{
				val = TOF3D_POSSIBLE_ITEM;
				obst_cnts[nearness] += 1;
			}
			else
			{
				int diffsum = 0;

				for(int ix=-1; ix<=1; ix++)
				{
					for(int iy=-1; iy<=1; iy++)
					{
						if(hmap_avgd[sx+ix][sy+iy] < 15)
							diffsum += hmap_avgd[sx+ix][sy+iy];
					}
				}

				if(diffsum > 90)
				{
					val = TOF3D_SMALL_ITEM;
					obst_cnts[nearness] += 4;
				}
				else if(diffsum > 70)
				{
					val = TOF3D_POSSIBLE_ITEM;
					obst_cnts[nearness] += 1;
				}
			}

			tof3ds[tof3d_wr].objmap[sy*TOF3D_HMAP_XSPOTS+sx] = val;
		}
	}

	pthread_mutex_lock(&cur_pos_mutex);
	tof3d_obstacle_levels[0] = obst_cnts[0];
	tof3d_obstacle_levels[1] = obst_cnts[1];
	tof3d_obstacle_levels[2] = obst_cnts[2];
	pthread_mutex_unlock(&cur_pos_mutex);


	if(_dbg_print)
	{
		//  --------------- PRINT ---------------------------
		for(int sx = TOF3D_HMAP_XSPOTS-1; sx >= 0; sx--)
		{
			printf("%4d ", sx*TOF3D_HMAP_SPOT_SIZE);

			for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
			{
				if(hmap[sx][sy] < -1000)
				{
					if(sy == TOF3D_HMAP_YSPOTS/2-260/TOF3D_HMAP_SPOT_SIZE || sy == TOF3D_HMAP_YSPOTS/2+260/TOF3D_HMAP_SPOT_SIZE)
						putchar('\'');
					else if(sy == TOF3D_HMAP_YSPOTS/2)
						putchar('\'');
					else
						putchar(' ');
				}
				else
				{
					int val = hmap[sx][sy];
					val/=10;
					if(val < -5) putchar('!');
					else if(val < -1) putchar(':');
					else if(val < 0) putchar('.');
					else if(val < 10) putchar('0' + val);
					else putchar('#');
				}

				//putchar(' ');
			}

			printf(" | ");

			for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
			{
				if(hmap_avgd[sx][sy] == -9999)
				{
					if(sy == TOF3D_HMAP_YSPOTS/2-260/TOF3D_HMAP_SPOT_SIZE || sy == TOF3D_HMAP_YSPOTS/2+260/TOF3D_HMAP_SPOT_SIZE)
						putchar('\'');
					else if(sy == TOF3D_HMAP_YSPOTS/2)
						putchar('\'');
					else
						putchar(' ');
				}
				else
				{
					int val = hmap_avgd[sx][sy];
					val/=10;
					if(val < -2) putchar('=');
					else if(val < 0) putchar('-');
					else if(val == 0) putchar(' ');
					else if(val < 10) putchar('0' + val);
					else putchar('#');
				}

				//putchar(' ');
			}

			printf(" | ");

			for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
			{
				int8_t val = tof3ds[tof3d_wr].objmap[sy*TOF3D_HMAP_XSPOTS+sx];
				if(val == TOF3D_DROP)
					putchar('X');
				else if(val == TOF3D_POSSIBLE_DROP)
					putchar('x');
				else if(val == TOF3D_WALL)
					putchar('#');
				else if(val == TOF3D_BIG_ITEM)
					putchar('O');
				else if(val == TOF3D_SMALL_ITEM)
					putchar('o');
				else if(val == TOF3D_POSSIBLE_ITEM)
					putchar('-');
				else if(sy == TOF3D_HMAP_YSPOTS/2-260/TOF3D_HMAP_SPOT_SIZE || sy == TOF3D_HMAP_YSPOTS/2+260/TOF3D_HMAP_SPOT_SIZE)
					putchar('\'');
				else if(sy == TOF3D_HMAP_YSPOTS/2)
					putchar('\'');
				else if(val == TOF3D_UNSEEN)
					putchar('.');
				else
					putchar(' ');
			}
			printf("\n");
		}
		printf("out-of-area ignored %d	confidence ignored %d\n", ignored, confi_ignored);
	}

	tof3d_wr++; if(tof3d_wr >= TOF3D_RING_BUF_LEN) tof3d_wr = 0;

}

Softkinetic_tof tof_instance;

extern "C" void* start_tof(void*);
void* start_tof(void* calibrate)
{
	int calib = *(uint8_t*)calibrate;
	if(!calib)
	{
		printf("tof3d: opening tof_zcalib.raw Z axis calibration file for read.\n");
		FILE* floor = fopen("/home/hrst/rn1-host/tof_zcalib.raw", "r");
		if(!floor)
		{
			printf("ERR: Couldn't open tof_zcalib.raw for read. Floor calibration is disabled.\n");
		}
		else
		{
			fread(hmap_calib, 2, TOF3D_HMAP_XSPOTS*TOF3D_HMAP_YSPOTS, floor);
			fclose(floor);
		}
	}

	tof_instance.init(calib);
	tof_instance.run();
	return NULL;
}

extern "C" void request_tof_quit(void);
void request_tof_quit(void)
{
	quit = true;
	tof_instance.do_quit();	
}

