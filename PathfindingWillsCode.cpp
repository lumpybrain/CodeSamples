/* Note from posting to Code Samples:
 * This cpp is from my junior year pathfinding project.
 * I wrote the A* pathfinding algorithm, and simplication of
 * the path (details in the presentation @
 * https://www.willpritz.com/projects/navmesh)
 * 
 * This CPP is not complete, as code from another
 * student has been stripped out.
 *
 * Sometimes notes are added to explain the code for posterity.
 * these are marked as Code Sample Notes (CSN)
 */

#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#define LEFT 1
#define RIGHT 2
#define PARALLEL 0

// Initialize the pather
bool AStarPather::initialize()
{
  // Register InitializeMap as a callback so it gets called whenever a map is loaded
  Callback cb = std::bind(&AStarPather::InitializeMap, this);
  Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

  return true; // Return false if any errors actually occur, to stop engine initialization
}

// Shut down the pather
void AStarPather::shutdown()
{
  
}

float sign (Vec3 p1, Vec3 p2, Vec3 p3)
{
    return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
}

bool PointInTriangle (Vec3 pt, Vec3 v1, Vec3 v2, Vec3 v3)
{
    float d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(pt, v1, v2);
    d2 = sign(pt, v2, v3);
    d3 = sign(pt, v3, v1);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

float distanceSquared(float startX, float startY, float endX, float endY)
{
    float xDiff = endX - startX;
    float yDiff = endY - startY;
    return sqrtf((xDiff * xDiff) + (yDiff * yDiff));
}

void AStarPather::remove_from_open(int pos)
{
    Triangle* holder = openList.back();
    openList.back() = openList[pos];
    openList[pos] = holder;
    openList.pop_back();
}

void AStarPather::remove_from_closed(int pos)
{
    Triangle* holder = closedList.back();
    closedList.back() = closedList[pos];
    closedList[pos] = holder;
    closedList.pop_back();
}


Vec3 AStarPather::TriangleCenter(Triangle& given)
{
    float x = (given.pointA.x + given.pointB.x + given.pointC.x) / 3.0f;
    float z = (given.pointA.z + given.pointB.z + given.pointC.z) / 3.0f;

    return Vec3(x, 0, z);
}

void AStarPather::addPathEdge(Triangle* given, std::list<Edge>& edges)
{
    // pushes the relevant edge to the edge vector (note; reverse edge direction for purposes of funnel algorithm)
    if (given != NULL && given->Parent != NULL)
    {
        if (given->Parent == given->neighborAB)
        {
            edges.push_front(Edge(given->pointB, given->pointA, given->neighborAB, given));
        }
        if (given->Parent == given->neighborBC)
        {
            edges.push_front(Edge(given->pointC, given->pointB, given->neighborBC, given));
        }
        if (given->Parent == given->neighborCA)
        {
            edges.push_front(Edge(given->pointA, given->pointC, given->neighborCA, given));
        }
    }
}

// use dot product to find if point A is clockwise or counterclockwise on the line
// from the origin to point B
int AStarPather::isPointALeft(Vec3 origin, Vec3 PointA, Vec3 PointB)
{
    Vec3 AVector = PointA - origin;
    Vec3 BVector = PointB - origin;
    // equivalent of a.x*-b.y + a.y*b.x
    float dot = AVector.z * -(BVector.x) + AVector.x * BVector.z;
    if (dot > 0)
        return LEFT;
    else if (dot < 0)
        return RIGHT;
    else
        return PARALLEL;
}

// CSN: Bugtesting function used to test the validity of pathfinding on the navmesh.
void AStarPather::simpleFinalizePath(Triangle* cheapest, PathRequest& request)
{
    request.path.push_front(request.goal);
    cheapest = cheapest->Parent;

    while (cheapest != NULL && cheapest->Parent != NULL)
    {
        request.path.push_front(TriangleCenter(*cheapest));
        cheapest = cheapest->Parent;
    }

    request.path.push_front(request.start);
}

// CSN: This is the meat of the project, taking a list of triangles
// that represents a path and simplifying the path as much as possible.

// finalize the found A* path
void AStarPather::finalizePath(Triangle* cheapest, PathRequest& request)
{
    std::list<Edge> edges;
    std::vector<Vec3> tail;
    std::vector<Vec3> leftVector;
    std::vector<Vec3> rightVector;
    addPathEdge(cheapest, edges);
    cheapest = cheapest->Parent;

    // get a list of all the edges from the found path
    while (cheapest != NULL && cheapest->Parent != NULL)
    {
        addPathEdge(cheapest, edges);
        cheapest = cheapest->Parent;
    }
    tail.push_back(request.start);

    // for every edge...
    for (auto it = edges.begin(); it != edges.end(); ++it)
    {
        int isLeft = isPointALeft(tail.back(), it->pointA, it->pointB);
        Vec3 leftPoint;
        Vec3 rightPoint;
        // figure out which point is left, and which point is right
        if (isLeft == LEFT)
        {
            leftPoint = it->pointA; rightPoint = it->pointB;
        }
        else if (isLeft == RIGHT)
        {
            leftPoint = it->pointB; rightPoint = it->pointA;
        }
        else if (isLeft == PARALLEL)
        {
            continue;
        }
        // dealing with the left side ///////////////////////////////////////////////////////////////////////////////////////////

        // if the left side is empty, just add the point
        if (leftVector.empty())
        {
            leftVector.push_back(leftPoint);
        }
        // if the left side is the back of the right vector, just add the entire right vector to the path
        else if (leftPoint == rightVector.back())
        {
            for (int i = 0; i < rightVector.size(); ++i)
            {
                tail.push_back(rightVector[i]);
            }
            leftVector.clear();
            rightVector.clear();
            continue;
        }
        // otherwise, if the left point isn't already on the back of the vector...
        else if(leftPoint != leftVector.back())
        {
            bool shifted = false;
            // if the left point is further to the right than any previous left point,
            // change the end of the vector to the left point
            for (int i = (int)leftVector.size() - 1; i >= 0; --i)
            {
                if (isPointALeft(tail.back(), leftPoint, leftVector[i]) != LEFT)
                {
                    leftVector[i] = leftPoint;
                    leftVector.resize((size_t)i + 1);
                    shifted = true;
                    if (!rightVector.empty())
                    {
                        bool testing = false;
                        while (!rightVector.empty() && isPointALeft(tail.back(), leftPoint, rightVector.front()) == RIGHT)
                        {
                            testing = true;
                            tail.push_back(rightVector.front());
                            std::vector<Vec3> holder;
                            for (int i = 1; i < rightVector.size(); ++i)
                            {
                                holder.push_back(rightVector[i]);
                            }
                            rightVector = holder;
                        }
                        if (testing)
                        {
                            leftVector.clear();
                            leftVector.push_back(leftPoint);
                            break;
                        }
                    }
                }
            }
            // if the left point is not further right than anything, just add it to
            // the left side
            if(shifted == false)
            {
                leftVector.push_back(leftPoint);
            }
        }

        // dealing with the right side //////////////////////////////////////////////////////////////////////////////////////////

        // if the right side is empty, just add the point
        if (rightVector.empty())
        {
            rightVector.push_back(rightPoint);
        }
        // if the right side is the back of the left vector, just add the entire right vector to the path
        else if (rightPoint == leftVector.back())
        {
            for (int i = 0; i < leftVector.size(); ++i)
            {
                tail.push_back(leftVector[i]);
            }
            leftVector.clear();
            rightVector.clear();
            continue;
        }
        // otherwise, if the left point isn't already on the back of the vector...
        else if(rightPoint != rightVector.back())
        {
            bool shifted = false;
            // if the right point is further to the left than any previous right point,
            // change the end of the vector to the right point
            for (int i = (int)rightVector.size() - 1; i >= 0; --i)
            {
                if (isPointALeft(tail.back(), rightPoint, rightVector[i]) != RIGHT)
                {
                    rightVector[i] = rightPoint;
                    rightVector.resize((size_t)i + 1);
                    shifted = true;
                    if (!leftVector.empty())
                    {
                        bool testing = false;
                        while (!leftVector.empty() && isPointALeft(tail.back(), rightPoint, leftVector.front()) == LEFT)
                        {
                            testing = true;
                            tail.push_back(leftVector.front());
                            std::vector<Vec3> holder;
                            for (int i = 1; i < leftVector.size(); ++i)
                            {
                                holder.push_back(leftVector[i]);
                            }
                            leftVector = holder;
                        }
                        if (testing)
                        {
                            rightVector.clear();
                            rightVector.push_back(rightPoint);
                            break;
                        }
                    }

                }
            }
            // if the right point is not further left than anything, just add it to
            // the right side
            if (shifted == false)
            {
                rightVector.push_back(rightPoint);
            }
        }

        // edge clean up and edge conditions ////////////////////////////////////////////////////////////////////////////////////
        if (leftVector.empty() || rightVector.empty())
        {
            continue;
        }
        if (leftVector.size() > 1 && isPointALeft(tail.back(), leftVector[leftVector.size() - 2], rightVector.back()) == RIGHT)
        {
            Vec3 holderLeft = leftVector.back(); Vec3 holderRight = rightVector.back();
            for (int i = 0; i < leftVector.size() - 1; ++i)
            {
                tail.push_back(leftVector[i]);
            }
            leftVector.clear(); rightVector.clear();
            leftVector.push_back(holderLeft); rightVector.push_back(holderRight);
        }
        else if (rightVector.size() > 1 && isPointALeft(tail.back(), rightVector[rightVector.size() - 2], leftVector.back()) == LEFT)
        {
            Vec3 holderLeft = leftVector.back(); Vec3 holderRight = rightVector.back();
            for (int i = 0; i < leftVector.size() - 1; ++i)
            {
                tail.push_back(rightVector[i]);
            }
            leftVector.clear(); rightVector.clear();
            leftVector.push_back(holderLeft); rightVector.push_back(holderRight);
        }
        else if (leftVector.back() == rightVector.front())
        {
            for (int i = 0; i < leftVector.size(); ++i)
            {
                tail.push_back(leftVector[i]);
            }
            leftVector.pop_back();
            rightVector.clear();
        }
        else if (rightVector.back() == leftVector.front())
        {
            for (int i = 0; i < rightVector.size(); ++i)
            {
                tail.push_back(rightVector[i]);
            }
            rightVector.pop_back();
            leftVector.clear();
        }
    }

    //check for final point
    bool substituted = false;
    for (int i = (int)rightVector.size() - 1; i >= 0; --i)
    {
        if (isPointALeft(tail.back(), request.goal, rightVector[i]) != LEFT)
        {
            for (int j = 0; j <= i; ++j)
            {
                tail.push_back(rightVector[j]);
            }
            substituted = true;
            break;
        }
    }
    if (!substituted)
    {
        for (int i = (int)leftVector.size() - 1; i >= 0; --i)
        {
            if (isPointALeft(tail.back(), request.goal, leftVector[i]) != RIGHT)
            {
                for (int j = 0; j <= i; ++j)
                {
                    tail.push_back(leftVector[j]);
                }
                break;
            }
        }
    }
    for (int i = 0; i < tail.size(); ++i)
    {
        request.path.push_back(tail[i]);
    }
    request.path.push_back(request.goal);
}

// CSN: This is basically just A* pathfinding, but on a navmesh of triangles, finding
// a path from triangle to triangle.

// Use A* pathing to find a path from an origin point to an end point
PathResult AStarPather::compute_path(PathRequest &request)
{
  if (request.newRequest)
  {
      // initialize
      for (auto it = navmesh.begin(); it != navmesh.end(); ++it)
      {
        it->Parent = NULL;
        it->totalCost_ = 0;
        it->givenCost_ = 0;
      }

      openList.clear();
      closedList.clear();
      startTriangle = nullptr;
      endTriangle = nullptr;
      float minStartDist = -1.f;
      Triangle *closestStart = nullptr;

      // find the start and end triangles for our path
      for (auto it = navmesh.begin(); it != navmesh.end(); ++it)
      {
          // check to find the triangle that our starting point is in
          if (startTriangle == nullptr)
          {
              if (PointInTriangle(request.start, it->pointA, it->pointB, it->pointC))
              {
                  openList.push_back(&(*it));
                  startTriangle = &(*it);
              }
              else
              {
                Vec3 triangleCenter = TriangleCenter(*it);
                float distToCenter = distanceSquared(request.start.x, request.start.z, triangleCenter.x, triangleCenter.z);

                if (closestStart == nullptr || distToCenter < minStartDist)
                {
                  minStartDist = distToCenter;
                  closestStart = &(*it);
                }
              }
          }

          // check to find the triangle that our ending point is in
          if (endTriangle == nullptr && PointInTriangle(request.goal, it->pointA, it->pointB, it->pointC))
          {
              endTriangle = &(*it);
          }

          // if we've found our start and end, break out
          if ((startTriangle != nullptr || closestStart != nullptr) && endTriangle != nullptr) break;
      }

      if (endTriangle == nullptr)
      {
        return PathResult::IMPOSSIBLE;
      }

      if (startTriangle == nullptr)
      {
        if (closestStart != nullptr)
        {
          startTriangle = closestStart;
          openList.push_back(startTriangle);
        }
        else
        {
          return PathResult::IMPOSSIBLE;
        }
      }

      Vec3 center = TriangleCenter(*startTriangle);
      startTriangle->totalCost_ = distanceSquared(center.x, center.z, request.goal.x, request.goal.z);
    }

  // while the open list has nodes to process
  while (!openList.empty())
  {
      // find the cheapest node in the open list
      Triangle* cheapest = openList[0];
      int cheapestPos = 0;
      for (size_t i = 0; i < openList.size(); ++i)
      {
          if (openList[i]->totalCost_ < cheapest->totalCost_)
          {
              cheapest = openList[i];
              cheapestPos = (int)i;
          }
      }

      Vec3 cheapestCenter = TriangleCenter(*cheapest);

      // if the cheapest node is the goal, return
      if (cheapest == endTriangle)
      {
          finalizePath(cheapest, request);
          return PathResult::COMPLETE;
      }
      // pop the cheapest node off the open list
      remove_from_open((int)cheapestPos);

      // for each of the Triangles children...
      for (int i = 0; i < 3; ++i)
      {
          Triangle *currentIndex = nullptr;
          switch (i)
          {
          case 0:
              currentIndex = cheapest->neighborAB;
              break;
          case 1:
              currentIndex = cheapest->neighborBC;
              break;
          case 2:
              currentIndex = cheapest->neighborCA;
              break;
          }
          if (currentIndex == nullptr) { continue; }
          Triangle *curr = currentIndex;
          // if this node was the previous in the path, bug out
          if (curr == cheapest->Parent) { continue; }

          // calculate the child nodes cost for this path

          Vec3 currCenter = TriangleCenter(*curr);
          float givenCost = cheapest->givenCost_;
          float currCost = distanceSquared(currCenter.x, currCenter.z, request.goal.x, request.goal.z) * request.settings.weight;
          givenCost += distanceSquared(cheapestCenter.x, cheapestCenter.z, currCenter.x, currCenter.z);
          currCost += givenCost;
          size_t cheapPos;

          // if the child node isn't on a list, put it on the open list
          if (curr->totalCost_ == 0.0f)
          {
              curr->totalCost_ = currCost;
              curr->givenCost_ = givenCost;
              curr->Parent = cheapest;
              openList.push_back(curr);
          }
          // if the child node is on a list and is cheaper, move it onto the open list
          else
          {
              // if its on the open list...
              bool found = false;
              for (cheapPos = 0; cheapPos < openList.size(); ++cheapPos)
              {
                  if (curr == openList[cheapPos] && currCost < curr->totalCost_)
                  {
                      curr->totalCost_ = currCost;
                      curr->givenCost_ = givenCost;
                      curr->Parent = cheapest;
                      found = true;
                      break;
                  }
              }
              // otherwise, if its on the closed list...
              if (found == false)
              {
                  for (cheapPos = 0; cheapPos < closedList.size(); ++cheapPos)
                  {
                      if (curr == closedList[cheapPos] && currCost < curr->totalCost_)
                      {
                          curr->totalCost_ = currCost;
                          curr->givenCost_ = givenCost;
                          curr->Parent = cheapest;
                          remove_from_closed((int)cheapPos);
                          openList.push_back(curr);
                          found = true;
                          break;
                      }
                  }
              }
          }
      }
      // put the parent node onto the closed list
      closedList.push_back(cheapest);
  }
  return PathResult::IMPOSSIBLE;
}