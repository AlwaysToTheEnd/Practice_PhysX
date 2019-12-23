#ifdef RENDER_SNIPPET

#include <vector>
#include <atomic>

#include "PxPhysicsAPI.h"
#include "SnippetRender.h"
#include "SnippetCamera.h"

using namespace physx;

extern void InitPhysics(bool interactive);
extern void StepPhysics(bool interactive);
extern void CleanupPhysics(bool interactive);

extern std::vector<PxVec3> gContactPositions;
extern std::vector<PxVec3> gContactImpulses;
extern std::vector<PxVec3> gContactVertices;
extern std::atomic_int32_t gSharedIndex;

namespace
{
	Snippets::Camera* sCamera;

	void motionCallback(int x, int y)
	{
		sCamera->handleMotion(x, y);
	}

	void keyboardCallback(unsigned char key, int x, int y)
	{
		if (key == 27)
			exit(0);

		sCamera->handleKey(key, x, y);
	}

	void mouseCallback(int button, int state, int x, int y)
	{
		sCamera->handleMouse(button, state, x, y);
	}

	void idleCallback()
	{
		glutPostRedisplay();
	}

	void renderCallback()
	{
		StepPhysics(true);

		Snippets::startRender(sCamera->getEye(), sCamera->getDir());

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
		}

		PxI32 count = gSharedIndex;
		if (count)
		{
			glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, 0, &gContactVertices[0]);
			glDrawArrays(GL_LINES, 0, GLint(count * 2));
			glDisableClientState(GL_VERTEX_ARRAY);
		}

		Snippets::finishRender();
	}

	void exitCallback(void)
	{
		delete sCamera;
		CleanupPhysics(true);
	}
}

void RenderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f, -0.2f, -0.7f));

	Snippets::setupDefaultWindow("PhysX Snippet Split fetchResults");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	motionCallback(0, 0);

	atexit(exitCallback);

	InitPhysics(true);
	glutMainLoop();
}
#endif
