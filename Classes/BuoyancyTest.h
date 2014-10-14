/*
 * BuoyancyTest.h
 *
 *  Created on: 08-Oct-2014
 *      Author: woigames
 */

#ifndef BUOYANCYTEST_H_
#define BUOYANCYTEST_H_

#include "GameGlobals.h"

class GLESDebugDraw;

class BuoyancyTest : public Layer, public b2ContactListener {
public:
	virtual ~BuoyancyTest();

	static Scene* createScene();
	virtual bool init();
    virtual void draw(Renderer *renderer, const Mat4 &transform, uint32_t flags) override;
    void onDraw(const Mat4 &transform, uint32_t flags);

	virtual void update(float dt);
	virtual void BeginContact(b2Contact* contact);
	virtual void EndContact(b2Contact* contact);

    virtual bool onTouchBegan(Touch* touch, Event* event);
	virtual void onTouchMoved(Touch* touch, Event* event);
	virtual void onTouchEnded(Touch* touch, Event* event);

	void CreateBuoyancyTest();
	void CreateFluid();
	void CreateTestBodies();
	void CreateBodyWithJoints();
//	void AddDynamicBody(Vec2 position, EBodySize size);
//	void AddStaticBody(Vec2 position, EBodySize size);

	void UpdateBuoyancy();
	void ApplyBuoyancyForce(b2Fixture* fixture_a, b2Fixture* fixture_b, float area, b2Vec2 centroid);
	void ApplyDrag(b2Fixture* fixture_a, b2Fixture* fixture_b, float area, b2Vec2 centroid);
	void UpdateTrajectory();

	CREATE_FUNC(BuoyancyTest);
    void menuCloseCallback(Ref* pSender);

private:
    b2Vec2 gravity_;
	b2World* world_;
	b2Body* base_;
	b2Body* fluid_;
	set<fixture_pair> fixture_pairs_;

	b2Body* player_body_;
    DrawNode* trajectory_;
    b2Vec2 impulse_;
    Vec2 last_touch_;

	GLESDebugDraw* debug_draw_;

    CustomCommand custom_command_;
};

#endif /* BUOYANCYTEST_H_ */
