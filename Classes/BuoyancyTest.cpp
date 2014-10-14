/*
 * BuoyancyTest.cpp
 *
 *  Created on: 08-Oct-2014
 *      Author: woigames
 */

#include "BuoyancyTest.h"
#include "GLES-Render.h"

BuoyancyTest::~BuoyancyTest() {
	// TODO Auto-generated destructor stub
}

Scene* BuoyancyTest::createScene()
{
	auto scene = Scene::create();
	auto layer = BuoyancyTest::create();
	scene->addChild(layer);
	return scene;
}

bool BuoyancyTest::init()
{
	if(!Layer::init())
		return false;

	gravity_ = b2Vec2_zero;
	world_ = nullptr;
	base_ = nullptr;
	fluid_ = nullptr;
	fixture_pairs_.clear();

	player_body_ = nullptr;
	trajectory_ = nullptr;
	impulse_ = b2Vec2_zero;
	last_touch_ = Vec2::ZERO;

	debug_draw_ = NULL;

    auto closeItem = MenuItemImage::create("CloseNormal.png", "CloseSelected.png", CC_CALLBACK_1(BuoyancyTest::menuCloseCallback, this));
	closeItem->setPosition(Vec2(SCREEN_SIZE.width - closeItem->getContentSize().width/2, closeItem->getContentSize().height/2));

    auto menu = Menu::create(closeItem, nullptr);
    menu->setPosition(Vec2::ZERO);
    this->addChild(menu, 1);

	CreateBuoyancyTest();

	auto listener = EventListenerTouchOneByOne::create();
	listener->onTouchBegan = CC_CALLBACK_2(BuoyancyTest::onTouchBegan, this);
	listener->onTouchMoved = CC_CALLBACK_2(BuoyancyTest::onTouchMoved, this);
	listener->onTouchEnded = CC_CALLBACK_2(BuoyancyTest::onTouchEnded, this);
	_eventDispatcher->addEventListenerWithSceneGraphPriority(listener, this);

	scheduleUpdate();
	return true;
}

void BuoyancyTest::CreateBuoyancyTest()
{
	gravity_.Set(0.0f, -9.8f);
	world_ = new b2World(gravity_);
	world_->SetContactListener(this);

	CreateFluid();
	CreateTestBodies();
	trajectory_ = DrawNode::create();
	addChild(trajectory_);

	// create debug draw
	debug_draw_ = new GLESDebugDraw(PTM_RATIO);
	uint32 flags = 0;
	flags += b2Draw::e_shapeBit;
	flags += b2Draw::e_jointBit;
	debug_draw_->SetFlags(flags);
	world_->SetDebugDraw(debug_draw_);
}

void BuoyancyTest::CreateFluid()
{
	// create base
	b2BodyDef base_body_def;
	base_body_def.type = b2_staticBody;
	base_ = world_->CreateBody(&base_body_def);

	b2ChainShape base_shape;
	b2Vec2 vertices[4] = { b2Vec2(0, SCREEN_TO_WORLD(SCREEN_SIZE.height/3)), b2Vec2(0, 0), b2Vec2(SCREEN_TO_WORLD(SCREEN_SIZE.width), 0), b2Vec2(SCREEN_TO_WORLD(SCREEN_SIZE.width), SCREEN_TO_WORLD(SCREEN_SIZE.height/3)) };
	base_shape.CreateChain(vertices, 4);
	base_->CreateFixture(&base_shape, 0);

	// create fluid
	b2BodyDef fluid_body_def;
	fluid_body_def.type = b2_staticBody;
	fluid_ = world_->CreateBody(&fluid_body_def);

	b2PolygonShape fluid_shape;
	fluid_shape.SetAsBox( SCREEN_TO_WORLD(SCREEN_SIZE.width/2), SCREEN_TO_WORLD(SCREEN_SIZE.height/6), b2Vec2(SCREEN_TO_WORLD(SCREEN_SIZE.width/2), SCREEN_TO_WORLD(SCREEN_SIZE.height/6)), 0 );
	b2FixtureDef fluid_fixture_def;
	fluid_fixture_def.shape = &fluid_shape;
	fluid_fixture_def.isSensor = true;
	fluid_fixture_def.density = 1.5f;
	fluid_->CreateFixture(&fluid_fixture_def);
}

void BuoyancyTest::CreateTestBodies()
{
	// create a base for player
	b2Body* test_body = GameGlobals::GetBodyForWorld(world_, b2_staticBody, b2Shape::e_polygon, E_BODY_MEDIUM);
	test_body->SetTransform( b2Vec2( SCREEN_TO_WORLD(SCREEN_SIZE.width * 0.5f), SCREEN_TO_WORLD(SCREEN_SIZE.height * 0.3f) ), 0 );
	test_body->GetFixtureList()->SetFriction(1.0f);

	// create player
	b2BodyDef ball_body_def;
	ball_body_def.type = b2_dynamicBody;
	player_body_ = world_->CreateBody(&ball_body_def);
	player_body_->SetTransform(b2Vec2(SCREEN_SIZE.width * 0.5f / PTM_RATIO, SCREEN_SIZE.height * 0.5f / PTM_RATIO), 0.0f);
	player_body_->SetFixedRotation(false);

	b2CircleShape ball_shape;
	ball_shape.m_radius = 0.2f;
//	b2PolygonShape ball_shape;
//	ball_shape.SetAsBox(0.2f, 0.2f);
	b2FixtureDef ball_fixture_def;
	ball_fixture_def.friction = 1.0f;
	ball_fixture_def.shape = &ball_shape;

	player_body_->CreateFixture(&ball_fixture_def);

	CreateBodyWithJoints();
}

void BuoyancyTest::CreateBodyWithJoints()
{
	// create the floating body
	b2Vec2 position;
	position.Set(SCREEN_TO_WORLD(SCREEN_SIZE.width * 0.8f), SCREEN_TO_WORLD(SCREEN_SIZE.height * 0.3f));
	b2Body* test_body = GameGlobals::GetBodyForWorld(world_, b2_dynamicBody, b2Shape::e_polygon, E_BODY_LARGE);
	test_body->SetTransform(position, 0);
	test_body->ApplyTorque(100.0f, true);

	// create two revolute joints to anchor the floating body
	b2DistanceJointDef distance_joint_def;
	b2Vec2 base_anchor;
	b2Vec2 body_anchor;

	base_anchor.Set(SCREEN_TO_WORLD(SCREEN_SIZE.width * 0.65f), 0.0f);
	body_anchor.Set(SCREEN_TO_WORLD(SCREEN_SIZE.width * 0.75f), SCREEN_TO_WORLD(SCREEN_SIZE.height * 0.2f));
	distance_joint_def.Initialize(base_, test_body, base_anchor, body_anchor);
	world_->CreateJoint(&distance_joint_def);

	base_anchor.Set(SCREEN_TO_WORLD(SCREEN_SIZE.width * 0.95f), 0.0f);
	body_anchor.Set(SCREEN_TO_WORLD(SCREEN_SIZE.width * 0.85f), SCREEN_TO_WORLD(SCREEN_SIZE.height * 0.2f));
	distance_joint_def.Initialize(base_, test_body, base_anchor, body_anchor);
	world_->CreateJoint(&distance_joint_def);
}

void BuoyancyTest::update(float dt)
{
	world_->Step(dt, 8, 3);
	UpdateBuoyancy();
}

void BuoyancyTest::BeginContact(b2Contact* contact)
{
	b2Fixture* fixture_a = contact->GetFixtureA();
	b2Fixture* fixture_b = contact->GetFixtureB();

	if(fixture_a->IsSensor() && fixture_b->GetBody()->GetType() == b2_dynamicBody)
		fixture_pairs_.insert(make_pair(fixture_a, fixture_b));
	else if(fixture_b->IsSensor() && fixture_a->GetBody()->GetType() == b2_dynamicBody)
		fixture_pairs_.insert(make_pair(fixture_b, fixture_a));
	else
	{
		if( (fixture_a->GetBody() == player_body_ && fixture_b->GetBody()->GetType() == b2_dynamicBody) ||
				(fixture_b->GetBody() == player_body_ && fixture_a->GetBody()->GetType() == b2_dynamicBody) )
			player_body_->SetLinearVelocity(b2Vec2_zero);
	}
}

void BuoyancyTest::EndContact(b2Contact* contact)
{
	b2Fixture* fixture_a = contact->GetFixtureA();
	b2Fixture* fixture_b = contact->GetFixtureB();

	if(fixture_a->IsSensor() && fixture_b->GetBody()->GetType() == b2_dynamicBody)
		fixture_pairs_.erase(make_pair(fixture_a, fixture_b));
	else if(fixture_b->IsSensor() && fixture_a->GetBody()->GetType() == b2_dynamicBody)
		fixture_pairs_.erase(make_pair(fixture_b, fixture_a));
}

void BuoyancyTest::UpdateBuoyancy()
{
	set<fixture_pair>::iterator it = fixture_pairs_.begin();
	set<fixture_pair>::iterator end = fixture_pairs_.end();
	while (it != end) {

		//fixtureA is the fluid
		b2Fixture* fixture_a = it->first;
		b2Fixture* fixture_b = it->second;

		float density = fixture_a->GetDensity();

		vector<b2Vec2> intersectionPoints;
		if (GameGlobals::FindIntersectionOfFixtures(fixture_a, fixture_b, intersectionPoints)) {

			//find centroid
			float area = 0;
			b2Vec2 centroid = GameGlobals::ComputeCentroid(intersectionPoints, area);

			ApplyBuoyancyForce(fixture_a, fixture_b, area, centroid);
			ApplyDrag(fixture_a, fixture_b, area, centroid);
		}

		++it;
	}
}

void BuoyancyTest::ApplyBuoyancyForce(b2Fixture* fixture_a, b2Fixture* fixture_b, float area, b2Vec2 centroid)
{
	float displacedMass = fixture_a->GetDensity() * area;
	b2Vec2 buoyancyForce = displacedMass * -gravity_;
	fixture_b->GetBody()->ApplyForce(buoyancyForce, centroid, true);
}

void BuoyancyTest::ApplyDrag(b2Fixture* fixture_a, b2Fixture* fixture_b, float area, b2Vec2 centroid)
{
	b2Vec2 relative_velocity = fixture_b->GetBody()->GetLinearVelocityFromWorldPoint(centroid) - fixture_a->GetBody()->GetLinearVelocityFromWorldPoint(centroid);
	float relative_velocity_mag = relative_velocity.Normalize();

	// apply linear drag
	float drag_mag = fixture_a->GetDensity() * relative_velocity_mag * relative_velocity_mag;
	b2Vec2 drag_force = drag_mag * -relative_velocity;
	fixture_b->GetBody()->ApplyForce(drag_force, centroid, true);

	// apply angular drag
//	float angular_drag = area * -fixture_b->GetBody()->GetAngularVelocity();
//	fixture_b->GetBody()->ApplyTorque(angular_drag, true);
}

void BuoyancyTest::UpdateTrajectory()
{
	vector<Vec2> points;
	GameGlobals::GetTrajectory(player_body_->GetPosition(), impulse_, gravity_, points);
	trajectory_->clear();
	for(int i = 0; i < NUM_TRAJECTORY_STEPS; ++i)
		trajectory_->drawDot(points[i], 1, ccc4f(1, 1, 1, 1.0f - (float)i/(float)NUM_TRAJECTORY_STEPS));
}

bool BuoyancyTest::onTouchBegan(Touch* touch, Event* event)
{
	impulse_ = b2Vec2(5.0f, 5.0f);
	impulse_.x /= player_body_->GetMass();
	impulse_.y /= player_body_->GetMass();
	UpdateTrajectory();
	last_touch_ = touch->getLocation();
	return true;
}

void BuoyancyTest::onTouchMoved(Touch* touch, Event* event)
{
	Vec2 curr_touch = touch->getLocation();
	float angle = CC_DEGREES_TO_RADIANS( (last_touch_.x - curr_touch.x) / 5 );
	b2Vec2 modified_impulse(impulse_.x * cosf(angle) - impulse_.y * sinf(angle), impulse_.x * sinf(angle) + impulse_.y * cosf(angle));
	impulse_ = modified_impulse;
	UpdateTrajectory();
	last_touch_.set(curr_touch);
}

void BuoyancyTest::onTouchEnded(Touch* touch, Event* event)
{
	player_body_->ApplyLinearImpulse(impulse_, player_body_->GetWorldCenter(), true);
	impulse_ = b2Vec2_zero;
	last_touch_.set(Vec2::ZERO);
	trajectory_->clear();
}

void BuoyancyTest::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
    Layer::draw(renderer, transform, flags);

    custom_command_.init(_globalZOrder);
    custom_command_.func = CC_CALLBACK_0(BuoyancyTest::onDraw, this, transform, flags);
    renderer->addCommand(&custom_command_);
}

void BuoyancyTest::onDraw(const Mat4 &transform, uint32_t flags)
{
    Director* director = Director::getInstance();
    CCASSERT(nullptr != director, "Director is null when seting matrix stack");
    director->pushMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
    director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, transform);

    GL::enableVertexAttribs( cocos2d::GL::VERTEX_ATTRIB_FLAG_POSITION );
    world_->DrawDebugData();
    CHECK_GL_ERROR_DEBUG();

    director->popMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
}

void BuoyancyTest::menuCloseCallback(Ref* pSender)
{
	Director::getInstance()->replaceScene(BuoyancyTest::createScene());
}
