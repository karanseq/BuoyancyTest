#ifndef __HELLOWORLD_SCENE_H__
#define __HELLOWORLD_SCENE_H__

#include "GameGlobals.h"

class HelloWorld : public Layer
{
public:
    // there's no 'id' in cpp, so we recommend returning the class instance pointer
    static Scene* createScene();

    // Here's a difference. Method 'init' in cocos2d-x returns bool, instead of returning 'id' in cocos2d-iphone
    virtual bool init();  
    
    // a selector callback
    void menuCloseCallback(Ref* pSender);
    
    // implement the "static create()" method manually
    CREATE_FUNC(HelloWorld);

    void CreateBoundary();
	void CreateBall();

    void GenerateTrajectory(b2Vec2 starting_velocity);

    virtual void update(float dt);

    virtual bool onTouchBegan(Touch* touch, Event* event);
	virtual void onTouchMoved(Touch* touch, Event* event);
	virtual void onTouchEnded(Touch* touch, Event* event);

protected:
	Rect boundary_rect_;
    Vec2 last_touch_;
	b2World* world_;
	b2Body* ball_body_;
    DrawNode* ball_node_;
    DrawNode* trajectory_;
    b2Vec2 impulse_;
};

#endif // __HELLOWORLD_SCENE_H__
