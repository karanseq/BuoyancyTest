/*
 * GameGlobals.cpp
 *
 *  Created on: 08-Oct-2014
 *      Author: woigames
 */

#include "GameGlobals.h"

Size GameGlobals::screen_size_ = Size::ZERO;

void GameGlobals::Init()
{
	GameGlobals::screen_size_ = Director::getInstance()->getWinSize();
}

b2Body* GameGlobals::GetBodyForWorld(b2World* world, b2BodyType type, b2Shape::Type shape, EBodySize body_size)
{
	b2BodyDef body_def;
	body_def.type = type;
	b2Body* body = world->CreateBody(&body_def);

	b2PolygonShape box_shape = GameGlobals::GetRandomBoxShape(body_size);
	b2FixtureDef fixture_def;
	fixture_def.shape = &box_shape;
	fixture_def.density = 0.5f;
	body->CreateFixture(&fixture_def);
	return body;
}

b2PolygonShape GameGlobals::GetRandomBoxShape(EBodySize body_size)
{
	b2PolygonShape box_shape;
	float width = 0, height = 0;

	switch(body_size)
	{
	case E_BODY_SMALL:
		width = MIN_SHAPE_SIZE + CCRANDOM_0_1() * MAX_SHAPE_SIZE;
		width *= 1.25f;
		height = MIN_SHAPE_SIZE + CCRANDOM_0_1() * MAX_SHAPE_SIZE;
		break;
	case E_BODY_MEDIUM:
		width = 3.75f * MIN_SHAPE_SIZE + CCRANDOM_0_1() * MAX_SHAPE_SIZE;
		width *= 1.25f;
		height = 3.75f * MIN_SHAPE_SIZE + CCRANDOM_0_1() * MAX_SHAPE_SIZE;
		break;
	case E_BODY_LARGE:
		width = 7.5f * MIN_SHAPE_SIZE + CCRANDOM_0_1() * MAX_SHAPE_SIZE;
		width *= 1.25f;
		height = 7.5f * MIN_SHAPE_SIZE + CCRANDOM_0_1() * MAX_SHAPE_SIZE;
		break;
	}

	box_shape.SetAsBox(width, height);
	return box_shape;
}

b2CircleShape GameGlobals::GetRandomCircleShape()
{
	b2CircleShape circle_shape;
	circle_shape.m_radius = MIN_SHAPE_SIZE + CCRANDOM_0_1() * (MAX_SHAPE_SIZE - MIN_SHAPE_SIZE);
	return circle_shape;
}

void GameGlobals::GetTrajectory(b2Vec2 initial_position, b2Vec2 initial_velocity, b2Vec2 gravity, vector<Vec2> &points)
{
	points.clear();

	float t = 1 / 60.0f; 								// seconds per time step (at 60fps)
	b2Vec2 step_velocity = t * initial_velocity; 		// m/s
	b2Vec2 step_gravity = t * t * gravity; 				// m/s/s

	for(int i = 0; i < NUM_TRAJECTORY_STEPS; ++i)
	{
		b2Vec2 point_b2 = initial_position + i * step_velocity + 0.5f * (i*i+i) * step_gravity;
		Vec2 point(point_b2.x * PTM_RATIO, point_b2.y * PTM_RATIO);
		points.push_back(point);
	}
}

bool GameGlobals::Inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p)
{
	return (cp2.x-cp1.x)*(p.y-cp1.y) > (cp2.y-cp1.y)*(p.x-cp1.x);
}

b2Vec2 GameGlobals::Intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e)
{
	b2Vec2 dc(cp1.x - cp2.x, cp1.y - cp2.y);
	b2Vec2 dp(s.x - e.x, s.y - e.y);
	float n1 = cp1.x * cp2.y - cp1.y * cp2.x;
	float n2 = s.x * e.y - s.y * e.x;
	float n3 = 1.0 / (dc.x * dp.y - dc.y * dp.x);
	return b2Vec2((n1 * dp.x - n2 * dc.x) * n3, (n1 * dp.y - n2 * dc.y) * n3);
}

//http://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping#JavaScript
//Note that this only works when fB is a convex polygon, but we know all
//fixtures in Box2D are convex, so that will not be a problem
bool GameGlobals::FindIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& output_vertices)
{
//currently this only handles polygon vs polygon
	if (fA->GetShape()->GetType() != b2Shape::e_polygon
			|| fB->GetShape()->GetType() != b2Shape::e_polygon)
		return false;

	b2PolygonShape* polyA = (b2PolygonShape*) fA->GetShape();
	b2PolygonShape* polyB = (b2PolygonShape*) fB->GetShape();

//fill subject polygon from fixtureA polygon
	for (int i = 0; i < polyA->GetVertexCount(); i++)
		output_vertices.push_back(fA->GetBody()->GetWorldPoint(polyA->GetVertex(i)));

//fill clip polygon from fixtureB polygon
	vector<b2Vec2> clip_polygon;
	for (int i = 0; i < polyB->GetVertexCount(); i++)
		clip_polygon.push_back(fB->GetBody()->GetWorldPoint(polyB->GetVertex(i)));

	b2Vec2 cp1 = clip_polygon[clip_polygon.size() - 1];
	for (int j = 0; j < clip_polygon.size(); j++)
	{
		b2Vec2 cp2 = clip_polygon[j];
		if (output_vertices.empty())
			return false;

		vector<b2Vec2> input_list = output_vertices;
		output_vertices.clear();
		b2Vec2 s = input_list[input_list.size() - 1]; //last on the input list
		for (int i = 0; i < input_list.size(); i++)
		{
			b2Vec2 e = input_list[i];
			if (GameGlobals::Inside(cp1, cp2, e))
			{
				if (!GameGlobals::Inside(cp1, cp2, s))
				{
					output_vertices.push_back(GameGlobals::Intersection(cp1, cp2, s, e));
				}
				output_vertices.push_back(e);
			}
			else if (GameGlobals::Inside(cp1, cp2, s))
			{
				output_vertices.push_back(GameGlobals::Intersection(cp1, cp2, s, e));
			}
			s = e;
		}
		cp1 = cp2;
	}

	return !output_vertices.empty();
}

//Taken from b2PolygonShape.cpp
b2Vec2 GameGlobals::ComputeCentroid(vector<b2Vec2> vs, float& area)
{
	int count = (int) vs.size();
	b2Assert(count >= 3);

	b2Vec2 c;
	c.Set(0.0f, 0.0f);
	area = 0.0f;

	// pRef is the reference point for forming triangles.
	// Its location doesnt change the result (except for rounding error).
	b2Vec2 pRef(0.0f, 0.0f);

	const float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; ++i) {
		// Triangle vertices.
		b2Vec2 p1 = pRef;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float32 D = b2Cross(e1, e2);

		float32 triangle_area = 0.5f * D;
		area += triangle_area;

		// Area weighted centroid
		c += triangle_area * inv3 * (p1 + p2 + p3);
	}

	// Centroid
	if (area > b2_epsilon)
		c *= 1.0f / area;
	else
		area = 0;
	return c;
}
