/*
	This module connects to a SoftKinetic DS325 camera, processes the depthmap and forms an object map.

	Object map values:
	4	Tall obstacle, count as wall
	3	Large obstacle
	2	Small object. Could be a step, but most likely something to be avoided.
	1	Possibility of a small step; could be a false positive, too.
	0	Nothing special
	-1	Small hole or drop. Could be a false positive, too.
	-2	Significant hole or drop.

	The camera is expected to be mounted on the top of the robot, looking downwards, slightly tilted to
	show the front area of the robot.

	DS325 cannot be used as a forward-looking camera since it lacks dealiasing functionality, so that
	longer distances than the measurement range are aliased as short distances.

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
, _mode30(true)
, _temporal_smooth(true)
, _dbg_print(true)
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
	DepthSense::Device::Capabilities caps = device.getCapabilities();
	
	printf("--- Device added ----------------------------------\n");
	printf( " Model:		%s\n"
			" Capabilities: %s\n"
			" Serial:	   %s\n"
			, DepthSense::Device::Model_toString(model).c_str()
			, DepthSense::Device::Capabilities_toString(caps).c_str()
			, device.getSerialNumber().c_str()
			);
	printf("---------------------------------------------------\n\n");

	device.nodeAddedEvent().connect(this, &Softkinetic_tof::onNodeAdded);
	device.nodeRemovedEvent().connect(this, &Softkinetic_tof::onNodeRemoved);
	
	_devices.push_back(device);
}


void Softkinetic_tof::onDeviceRemoved(Context context, Device device)
{
	printf("Device removed\n");
	_devices.remove(device);
}

void Softkinetic_tof::onNodeAdded(Device device, Node node)
{
	if (_error) return;

	int32_t width = 0;
	int32_t height = 0;
	
	printf("--- Node found ------------------------------------\n");
	
	printf( " Node type: %s\n"
			" VID:	   %04x\n"
			" PID:	   %04x\n"
			" Revision:  %04d\n"
			, node.getType().name().c_str()
			, node.getVID()
			, node.getPID()
			, node.getRevision()
	);

	if (node.is<DepthNode>())
	{
		printf(" is DepthNode\n");
		DepthNode n = node.as<DepthNode>(); 

		DepthNode::Configuration configuration = n.getConfiguration();

			printf("\n Available configurations:\n");
			std::vector<DepthNode::Configuration> configurations = n.getConfigurations();
			bool doSetConfiguration = false;
			for (unsigned int i = 0; i < configurations.size(); i++)
			{
				printf("	%s - %d fps - %s - saturation %s", DepthSense::FrameFormat_toString(configurations[i].frameFormat).c_str(),
															   configurations[i].framerate,
															   DepthNode::CameraMode_toString(configurations[i].mode).c_str(),
															   configurations[i].saturation ? "enabled" : "disabled");

				if(configurations[i].framerate == (_mode30?30:6) && strcmp(DepthNode::CameraMode_toString(configurations[i].mode).c_str(), "LongRange") == 0)
				{
					printf(" MATCHING CONFIG!\n");
					configuration = configurations[i];
					doSetConfiguration = true;
				}

				printf("\n");
			}
			printf("\n");

			n.setEnableDepthMap(true);
			n.setEnableConfidenceMap(true);
			if (doSetConfiguration)
			{

				try {
					_context.requestControl(n, 0);
				}

				catch (Exception&)
				{
					printf("---------------------------------------------------\n\n");
					printf("Error : Could not take control\n");
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
					printf("---------------------------------------------------\n\n");
					printf("Incorrect configuration :\n");
					printf(" - Frame format: %s\n", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str());
					printf(" - Frame rate: %d fps\n", configuration.framerate);
					printf(" - Mode: %s\n", DepthNode::CameraMode_toString(configuration.mode).c_str());
					printf(" - Saturation: %s\n", configuration.saturation ? "enabled" : "disabled");

					_error = true;
					CONTEXT_QUIT(_context);
					return;
				}
			}

			printf("\n Depth node streaming enabled - current configuration:\n");
			printf("	%s - %d fps - %s - saturation %s", DepthSense::FrameFormat_toString(configuration.frameFormat).c_str(),
														   configuration.framerate,
														   DepthNode::CameraMode_toString(configuration.mode).c_str(),
														   configuration.saturation ? "enabled" : "disabled");
			printf("\n");

			n.newSampleReceivedEvent().connect
				(this, &Softkinetic_tof::onNewDepthNodeSampleReceived);

		DepthSense::FrameFormat_toResolution(configuration.frameFormat, &width, &height);

		printf("---------------------------------------------------\n\n");
	
		try
		{
			_context.registerNode(node);
		}
		catch(Exception& e)
		{
			fprintf(stderr, "Could not register node : %s", e.what());
			exit(-1);
		}

		try
		{
			_context.startNodes();
			_streaming = true;
		}
		catch(Exception& e)
		{
			fprintf(stderr, "Couldn't start streaming : %s", e.what());
			exit(-1);
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
	printf("Node removed\n");
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

#define HMAP_SPOT_SIZE 40
#define HMAP_YSPOTS 40
#define HMAP_YMIDDLE 20
#define HMAP_XSPOTS 30
#define HMAP_TEMPO 30

static int16_t hmap_calib[HMAP_XSPOTS][HMAP_YSPOTS];

void Softkinetic_tof::onNewDepthNodeSampleReceived(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
	const int16_t* depthMap = (const int16_t*) data.depthMap;
	const int16_t* confiMap = (const int16_t*) data.confidenceMap;


	if (!depthMap || !confiMap)
	{
		printf("ERROR: depthMap or confiMap NULL\n");
		return;
	}

	int32_t hmap_accum[HMAP_XSPOTS][HMAP_YSPOTS];
	int16_t hmap_nsamples[HMAP_XSPOTS][HMAP_YSPOTS];
	int16_t hmap[HMAP_XSPOTS][HMAP_YSPOTS];
	static int32_t hmap_temporal_acc[HMAP_XSPOTS][HMAP_YSPOTS];
	static int hmap_temporal_idx = 0;
	static int32_t hmap_calib_accum[HMAP_XSPOTS][HMAP_YSPOTS];
	static int16_t hmap_calib_cnt[HMAP_XSPOTS][HMAP_YSPOTS];
	int16_t hmap_avgd[HMAP_XSPOTS][HMAP_YSPOTS];
	static int8_t objmap[HMAP_XSPOTS][HMAP_YSPOTS];
	for(int xx=0; xx < HMAP_XSPOTS; xx++) { for(int yy=0; yy < HMAP_YSPOTS; yy++)
		{hmap_accum[xx][yy] = -9999; hmap_nsamples[xx][yy] = 0; hmap_avgd[xx][yy] = 0;} }

	const float ang_per_pixel = (72.44/*deg horizontal fov*/ /320.0)/360.0*2.0*M_PI;
	const float top_cam_ang = (32.0)/360.0*2.0*M_PI;

	int ignored = 0;
	int confi_ignored = 0;

	for(int py=2; py < 240-2; py+=2)
	{
		for(int px=0+2; px < 320-2; px+=2)
		{
			float pxang  = (float)(px-160) * ang_per_pixel;
			float pyang  = (float)(120-py) * ang_per_pixel;

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

			if(d < 0 || d > 1800) continue;

			float x = d * sin(pyang+top_cam_ang) + 70.0;
			float y = d * sin(pxang);
			float z = -1.0 * d * (1.0/cos(pyang)) * cos(pyang+top_cam_ang) + 900.0;

			int xspot = (int)(x / (float)HMAP_SPOT_SIZE);
			int yspot = (int)(y / (float)HMAP_SPOT_SIZE) + HMAP_YMIDDLE;

			if(xspot < 0 || xspot >= HMAP_XSPOTS || yspot < 0 || yspot >= HMAP_YSPOTS)
			{
				ignored++;
				continue;
			}

			if(z > hmap_accum[xspot][yspot]) hmap_accum[xspot][yspot] = z;

			hmap_nsamples[xspot][yspot]++;
		}
	}

	for(int sx = 0; sx < HMAP_XSPOTS; sx++)
	{
		for(int sy = 0; sy < HMAP_YSPOTS; sy++)
		{
			if(hmap_nsamples[sx][sy] > 0)
			{
				hmap[sx][sy] = hmap_accum[sx][sy];
				if(_calibrating)
				{
					hmap_calib_accum[sx][sy] += hmap[sx][sy];
					hmap_calib_cnt[sx][sy]++;
				}
				else
				{
					hmap[sx][sy] -= hmap_calib[sx][sy];
				}

			}
			else
				hmap[sx][sy] = -9999;

		}
	}

	if(_calibrating)
	{
		static int calib_cnt = 0;
		calib_cnt++;
		if(calib_cnt > 500)
		{
			FILE* floor = fopen("floor.raw", "w");
			for(int sx = 0; sx < HMAP_XSPOTS; sx++)
			{
				for(int sy = 0; sy < HMAP_YSPOTS; sy++)
				{
					if(hmap_calib_cnt[sx][sy] < 50)
						hmap_calib[sx][sy] = 0;
					else
						hmap_calib[sx][sy] = hmap_calib_accum[sx][sy] / hmap_calib_cnt[sx][sy];
				}
			}
			fwrite(hmap_calib, 2, HMAP_XSPOTS*HMAP_YSPOTS, floor);
			fclose(floor);
			CONTEXT_QUIT(_context);
		}
	}

	if(_temporal_smooth)
	{
		for(int sx = 0; sx < HMAP_XSPOTS; sx++)
		{
			for(int sy = 0; sy < HMAP_YSPOTS; sy++)
			{
				hmap_temporal_acc[sx][sy] += hmap[sx][sy];
			}
		}
		hmap_temporal_idx++;

		if(hmap_temporal_idx >= HMAP_TEMPO)
		{
			for(int sx = 0; sx < HMAP_XSPOTS; sx++)
			{
				for(int sy = 0; sy < HMAP_YSPOTS; sy++)
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

	for(int sx = 3; sx < HMAP_XSPOTS-3; sx++)
	{
		for(int sy = 3; sy < HMAP_YSPOTS-3; sy++)
		{
			int nsamp = 0;
			int acc = 0;

			for(int ix=-6; ix<=6; ix++)
			{
				for(int iy=-6; iy<=6; iy++)
				{
					if(sx+ix < 0 || sx+ix > HMAP_XSPOTS-1 || sy+iy < 0 || sy+iy > HMAP_YSPOTS-1)
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

	// ------------ GENERATE OBJMAP BASED ON AVGD -------------

	for(int sx = 2; sx < HMAP_XSPOTS-2; sx++)
	{
		for(int sy = 2; sy < HMAP_YSPOTS-2; sy++)
		{

			if(hmap[sx][sy] < -998 || hmap_avgd[sx][sy] < -998)
				continue;

			if(hmap[sx][sy] < -200 || hmap_avgd[sx][sy] < -100)
			{
				objmap[sx][sy] = -2;
			}
			else if(hmap[sx][sy] < -150 || hmap_avgd[sx][sy] < -60)
			{
				objmap[sx][sy] = -1;
			}
			else if(hmap[sx][sy] > 250 || hmap_avgd[sx][sy] > 150)
			{
				objmap[sx][sy] = 4;
			}
			else if(hmap[sx][sy] > 150 || hmap_avgd[sx][sy] > 50)
			{
				objmap[sx][sy] = 3;
			}
			else if(hmap[sx][sy] > 120 || hmap_avgd[sx][sy] > 30)
			{
				objmap[sx][sy] = 2;
			}
			else if(hmap[sx][sy] > 100 || hmap_avgd[sx][sy] > 15)
			{
				objmap[sx][sy] = 1;
			}
			else
			{
				int diffsum = 0;

				for(int ix=-1; ix<=1; ix++)
				{
					for(int iy=-1; iy<=1; iy++)
					{
						if(hmap_avgd[sx+ix][sy+iy] < 30)
							diffsum += hmap_avgd[sx+ix][sy+iy];
					}
				}

				if(diffsum > 100)
					objmap[sx][sy] = 2;
				else if(diffsum > 80)
					objmap[sx][sy] = 1;
			}
		}
	}

	if(_dbg_print)
	{
		//  --------------- PRINT ---------------------------
		for(int sx = HMAP_XSPOTS-1; sx >= 0; sx--)
		{
			printf("%4d ", sx*HMAP_SPOT_SIZE);

			for(int sy = 0; sy < HMAP_YSPOTS; sy++)
			{
				if(hmap[sx][sy] < -1000)
				{
					if(sy == HMAP_YSPOTS/2-260/HMAP_SPOT_SIZE || sy == HMAP_YSPOTS/2+260/HMAP_SPOT_SIZE)
						putchar('\'');
					else if(sy == HMAP_YSPOTS/2)
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

			for(int sy = 0; sy < HMAP_YSPOTS; sy++)
			{
				if(hmap_avgd[sx][sy] == -9999)
				{
					if(sy == HMAP_YSPOTS/2-260/HMAP_SPOT_SIZE || sy == HMAP_YSPOTS/2+260/HMAP_SPOT_SIZE)
						putchar('\'');
					else if(sy == HMAP_YSPOTS/2)
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

			for(int sy = 0; sy < HMAP_YSPOTS; sy++)
			{
				if(objmap[sx][sy] == -2)
					putchar('X');
				else if(objmap[sx][sy] == -1)
					putchar('x');
				else if(objmap[sx][sy] == 4)
					putchar('#');
				else if(objmap[sx][sy] == 3)
					putchar('O');
				else if(objmap[sx][sy] == 2)
					putchar('o');
				else if(objmap[sx][sy] == 1)
					putchar('-');
				else if(sy == HMAP_YSPOTS/2-260/HMAP_SPOT_SIZE || sy == HMAP_YSPOTS/2+260/HMAP_SPOT_SIZE)
					putchar('\'');
				else if(sy == HMAP_YSPOTS/2)
					putchar('\'');
				else
					putchar(' ');

			}
			printf("\n");
		}
		printf("out-of-area ignored %d	confidence ignored %d\n", ignored, confi_ignored);
	}

}

extern "C" void* start_tof(void*);
void* start_tof(void* calibrate)
{
	int calib = *(uint8_t*)calibrate;
	if(!calib)
	{
		printf("INFO: tof3d: opening tof_zcalib.raw Z axis calibration file for read.\n");
		FILE* floor = fopen("/home/hrst/rn1-host/tof_zcalib.raw", "r");
		if(!floor)
		{
			printf("ERR: Couldn't open tof_zcalib.raw for read. Floor calibration is disabled.\n");
		}
		else
		{
			fread(hmap_calib, 2, HMAP_XSPOTS*HMAP_YSPOTS, floor);
			fclose(floor);
		}
	}

	Softkinetic_tof tof_instance;
	tof_instance.init(calib);
	tof_instance.run();
	return NULL;
}
