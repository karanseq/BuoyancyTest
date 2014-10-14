/*
 * GameGlobals.h
 *
 *  Created on: 08-Oct-2014
 *      Author: woigames
 */

#ifndef GAMEGLOBALS_H_
#define GAMEGLOBALS_H_

#include "cocos2d.h"
#include "Box2D/Box2D.h"

using namespace cocos2d;
using namespace std;

#define SCREEN_SIZE GameGlobals::screen_size_
#define PTM_RATIO 128
#define SCREEN_TO_WORLD(value) (value) / PTM_RATIO
#define WORLD_TO_SCREEN(value) (value) * PTM_RATIO

#define MIN_SHAPE_SIZE 0.1f
#define MAX_SHAPE_SIZE 0.25f
#define NUM_TRAJECTORY_STEPS 60
#define IMPULSE_MAGNITUDE

typedef pair<b2Fixture*, b2Fixture*> fixture_pair;

enum EBodySize
{
	E_BODY_SMALL = 0,
	E_BODY_MEDIUM,
	E_BODY_LARGE,
};

class GameGlobals {
public:
	GameGlobals(){};
	~GameGlobals(){};

	static void Init();

	static b2Body* GetBodyForWorld(b2World* world, b2BodyType type, b2Shape::Type shape, EBodySize body_size = E_BODY_MEDIUM);
	static b2PolygonShape GetRandomBoxShape(EBodySize body_size = E_BODY_MEDIUM);
	static b2CircleShape GetRandomCircleShape();
	static void GetTrajectory(b2Vec2 initial_position, b2Vec2 initial_velocity, b2Vec2 gravity, vector<Vec2> &points);

	// buoyancy related
	static bool Inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p);
	static b2Vec2 Intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e);
	static bool FindIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& output_vertices);
	static b2Vec2 ComputeCentroid(vector<b2Vec2> vs, float& area);

	static Size screen_size_;
};

#endif /* GAMEGLOBALS_H_ */
