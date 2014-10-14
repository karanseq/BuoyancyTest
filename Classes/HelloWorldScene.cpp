#include "HelloWorldScene.h"

Scene* HelloWorld::createScene()
{
    auto scene = Scene::create();
    auto layer = HelloWorld::create();
    scene->addChild(layer);
    return scene;
}

bool HelloWorld::init()
{
    if ( !Layer::init() )
    {
        return false;
    }
    
    boundary_rect_ = Rect(50, 50, SCREEN_SIZE.width - 100, SCREEN_SIZE.height - 100);
    last_touch_ = Vec2::ZERO;
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

    auto closeItem = MenuItemImage::create("CloseNormal.png", "CloseSelected.png", CC_CALLBACK_1(HelloWorld::menuCloseCallback, this));
    
	closeItem->setPosition(Vec2(origin.x + SCREEN_SIZE.width - closeItem->getContentSize().width/2, origin.y + closeItem->getContentSize().height/2));

    // create menu, it's an autorelease object
    auto menu = Menu::create(closeItem, nullptr);
    menu->setPosition(Vec2::ZERO);
    this->addChild(menu, 1);
    
    b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	world_ = new b2World(gravity);

    trajectory_ = DrawNode::create();
    addChild(trajectory_);

    CreateBoundary();
    CreateBall();

//    setTouchEnabled(true);

    auto listener = EventListenerTouchOneByOne::create();
    listener->onTouchBegan = CC_CALLBACK_2(HelloWorld::onTouchBegan, this);
    listener->onTouchMoved = CC_CALLBACK_2(HelloWorld::onTouchMoved, this);
    listener->onTouchEnded = CC_CALLBACK_2(HelloWorld::onTouchEnded, this);
    _eventDispatcher->addEventListenerWithSceneGraphPriority(listener, this);

    scheduleUpdate();
    return true;
}

void HelloWorld::CreateBoundary()
{
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0, 0);

	b2Body* groundBody = world_->CreateBody(&groundBodyDef);

	// Define the ground box shape.
	b2EdgeShape groundBox;

	// bottom
	groundBox.Set(b2Vec2(boundary_rect_.getMinX()/PTM_RATIO, boundary_rect_.getMinY()/PTM_RATIO), b2Vec2(boundary_rect_.getMaxX()/PTM_RATIO, boundary_rect_.getMinY()/PTM_RATIO));
	groundBody->CreateFixture(&groundBox,0);
	groundBox.Set(b2Vec2(boundary_rect_.getMinX()/PTM_RATIO, boundary_rect_.getMinY()/PTM_RATIO), b2Vec2(boundary_rect_.getMinX()/PTM_RATIO, boundary_rect_.getMaxY()/PTM_RATIO));
	groundBody->CreateFixture(&groundBox,0);
	groundBox.Set(b2Vec2(boundary_rect_.getMaxX()/PTM_RATIO, boundary_rect_.getMinY()/PTM_RATIO), b2Vec2(boundary_rect_.getMaxX()/PTM_RATIO, boundary_rect_.getMaxY()/PTM_RATIO));
	groundBody->CreateFixture(&groundBox,0);
}

void HelloWorld::CreateBall()
{
    // create body
    b2BodyDef ball_body_def;
    ball_body_def.type = b2_dynamicBody;
    ball_body_ = world_->CreateBody(&ball_body_def);
    ball_body_->SetTransform(b2Vec2(SCREEN_SIZE.width*0.125f / PTM_RATIO, SCREEN_SIZE.height*0.125f / PTM_RATIO), 0.0f);

    b2CircleShape ball_shape;
    ball_shape.m_radius = 0.2f;

    // create fixture
    b2FixtureDef ball_fixture_def;
    ball_fixture_def.shape = &ball_shape;
    ball_fixture_def.restitution = 0.5f;

    ball_body_->CreateFixture(&ball_fixture_def);

    ball_node_ = DrawNode::create();
    ball_node_->drawDot(CCPointZero, 25, ccc4f(1, 1, 1, 0.5f));
    addChild(ball_node_);
}

void HelloWorld::GenerateTrajectory(b2Vec2 starting_velocity)
{
	b2Vec2 gravity(0.0f, -10.0f);

	vector<Vec2> points;
	GameGlobals::GetTrajectory(ball_body_->GetPosition(), starting_velocity, gravity, points);

	trajectory_->clear();
	for(int i = 0; i < NUM_TRAJECTORY_STEPS; ++i)
		trajectory_->drawDot(points[i], 1, ccc4f(1, 1, 1, 1.0f - (float)i/(float)NUM_TRAJECTORY_STEPS));
}

void HelloWorld::update(float dt)
{
	world_->Step(dt, 8, 3);
	ball_node_->setPosition(ball_body_->GetPosition().x * PTM_RATIO, ball_body_->GetPosition().y * PTM_RATIO);
}

bool HelloWorld::onTouchBegan(Touch* touch, Event* event)
{
	impulse_ = b2Vec2(5.0f, 5.0f);	//b2Vec2(7.5f, 7.5f);
	impulse_.x /= ball_body_->GetMass();
	impulse_.y /= ball_body_->GetMass();

    GenerateTrajectory(impulse_);

    last_touch_ = touch->getLocationInView();

    return true;
}

void HelloWorld::onTouchMoved(Touch* touch, Event* event)
{
	Vec2 curr_touch = touch->getLocationInView();

	float angle = CC_DEGREES_TO_RADIANS( (last_touch_.x - curr_touch.x) / 5 );
	b2Vec2 modified_impulse(impulse_.x * cosf(angle) - impulse_.y * sinf(angle), impulse_.x * sinf(angle) + impulse_.y * cosf(angle));
	impulse_ = modified_impulse;
	GenerateTrajectory(impulse_);

	last_touch_.set(curr_touch);
}

void HelloWorld::onTouchEnded(Touch* touch, Event* event)
{
	ball_body_->ApplyLinearImpulse(impulse_, ball_body_->GetWorldCenter(), true);
	impulse_ = b2Vec2_zero;
	last_touch_.set(Vec2::ZERO);
	trajectory_->clear();
}

void HelloWorld::menuCloseCallback(Ref* pSender)
{
	Director::getInstance()->replaceScene(HelloWorld::createScene());
}
