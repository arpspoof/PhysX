
#include <ctype.h>
#include <vector>
#include <stdio.h>

#include "PxPhysicsAPI.h"
#include "config.h"
#include "globals.h"
#include "articulationTree.h"
#include "simbicon.h"

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
PxArticulationCache*					gCache = NULL;
PxArticulationJointReducedCoordinate*	gDriveJoint = NULL;

Articulation ar;

PxFilterFlags collisionShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* /*constantBlock*/, PxU32 /*constantBlockSize*/)
{
	// let triggers through
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}
	// generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where 
	// the filtermask of A contains the ID of B and vice versa.
	if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;

	return PxFilterFlag::eDEFAULT;
}

static int contactFlag = 0;

void onContactGround(PxU32 objGroup) {
	if (objGroup & CollisionGroup::LeftFoot) {
		contactFlag |= SIMBICON_LFOOT_CONTACT;
	}
	if (objGroup & CollisionGroup::RightFoot) {
		contactFlag |= SIMBICON_RFOOT_CONTACT;
	}
}

class CollisionCallback : public PxSimulationEventCallback {
	void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) override {
		PxU32 objGroup = 0;
		for (PxU32 i = 0; i < nbPairs; i++)
		{
			const PxContactPair& cp = pairs[i];

			if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
			{
				if (pairHeader.actors[0] == ar.linkMap["left_ankle"]->link ||
					pairHeader.actors[1] == ar.linkMap["left_ankle"]->link) {
					objGroup |= CollisionGroup::LeftFoot;
				}
				if (pairHeader.actors[0] == ar.linkMap["right_ankle"]->link ||
					pairHeader.actors[1] == ar.linkMap["right_ankle"]->link) {
					objGroup |= CollisionGroup::RightFoot;
				}
			}
		}
		if (objGroup != 0) {
			onContactGround(objGroup);
		}
	}

	void onConstraintBreak(PxConstraintInfo * /*constraints*/, PxU32 /*count*/) {}
	void onWake(PxActor ** /*actors*/, PxU32 /*count*/) {}
	void onSleep(PxActor ** /*actors*/, PxU32 /*count*/) {}
	void onTrigger(PxTriggerPair * /*pairs*/, PxU32 /*count*/) {}
	void onAdvance(const PxRigidBody *const * /*bodyBuffer*/, const PxTransform * /*poseBuffer*/, const PxU32 /*count*/) {}
};

CollisionCallback collisionCallback;

void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask)
{
	PxFilterData filterData;
	filterData.word0 = filterGroup; // word0 = own ID
	filterData.word1 = filterMask;  // word1 = ID mask to filter pairs that trigger a
									// contact callback;
	const PxU32 numShapes = actor->getNbShapes();
	PxShape** shapes = (PxShape**)malloc(sizeof(PxShape*)*numShapes);
	actor->getShapes(shapes, numShapes);
	for (PxU32 i = 0; i < numShapes; i++)
	{
		PxShape* shape = shapes[i];
		shape->setSimulationFilterData(filterData);
	}
	free(shapes);
}

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
	sceneDesc.filterShader	= collisionShader;

	sceneDesc.cudaContextManager = gCudaContextManager;

	sceneDesc.solverType = PxSolverType::eTGS;
	sceneDesc.simulationEventCallback = &collisionCallback;

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
		setupFiltering(groundPlane, CollisionGroup::Ground, CollisionGroup::LeftFoot | CollisionGroup::RightFoot);
		gScene->addActor(*groundPlane);
	}
	
	gArticulation = gPhysics->createArticulationReducedCoordinate();

	loader();

	gScene->addArticulation(*gArticulation);

	gCache = gArticulation->createCache();
	gArticulation->commonInit();

/*	gArticulation->copyInternalStateToCache(*gCache, PxArticulationCache::eALL);
	PxReal *vp = gCache->jointPosition;
	vp[9] = -0.5f; // +1
	vp[10] = 0.5f; // +0.5
	vp[11] = -0.5; // +2.5
	gArticulation->applyCache(*gCache, PxArticulationCache::eALL);*/

	initControl();
}

void stepPhysics(bool /*interactive*/)
{
	const PxReal dt = getConfigF("C_TIME_STEP");

	control(dt, contactFlag);
	contactFlag = 0;

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
