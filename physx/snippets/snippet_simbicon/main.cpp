
#include <ctype.h>
#include <vector>
#include <stdio.h>

#include "PxPhysicsAPI.h"
#include "config.h"
#include "globals.h"

#include "../snippetutils/SnippetUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation		= NULL;
PxPhysics*				gPhysics		= NULL;

PxDefaultCpuDispatcher*	gDispatcher		= NULL;
PxScene*				gScene			= NULL;

PxMaterial*				gMaterial		= NULL;

PxPvd*                  gPvd			= NULL;

PxArticulationReducedCoordinate*		gArticulation = NULL;
PxArticulationJointReducedCoordinate*	gDriveJoint = NULL;

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -getConfigF("C_GRAVITY"), 0.0f);

	PxCudaContextManagerDesc cudaContextManagerDesc;
	auto gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	
//	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
//	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);

	gDispatcher = PxDefaultCpuDispatcherCreate(getConfigI("C_THREADS"));

	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	sceneDesc.cudaContextManager = gCudaContextManager;

	sceneDesc.solverType = PxSolverType::eTGS;

	if (getConfigI("S_ENABLE_GPU_DYNAMICS")) {
		sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
		printf("S_ENABLE_GPU_DYNAMICS is enabled\n");
	}

	if (getConfigI("S_ENABLE_GPU_BROADPHASE")) {
		sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
		printf("S_ENABLE_GPU_BROADPHASE is enabled\n");
	}

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(getConfigF("C_STATIC_FRICTION"), getConfigF("C_DYNAMIC_FRICTION"), 0.f);

	if (getConfigI("S_GROUND")) {
		PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
		gScene->addActor(*groundPlane);
	}
	
	gArticulation = gPhysics->createArticulationReducedCoordinate();

	loader();

	gScene->addArticulation(*gArticulation);
}

void stepPhysics(bool /*interactive*/)
{
	const PxReal dt = getConfigF("C_TIME_STEP");
	control(dt);

	gScene->simulate(dt);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)
{
	gArticulation->release();
	gScene->release();
	gDispatcher->release();
	gPhysics->release();	
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	PxCloseExtensions();  
	gFoundation->release();

	printf("SnippetArticulation done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	keyHandler(key, camera);
}

int snippetMain(int argc, const char*const* argv)
{
	if (argc > 1) {
		const char* config_path = argv[1];
		readConfigFile(config_path);
	}
	else {
		printf("no config file specified\n");
	}

#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
