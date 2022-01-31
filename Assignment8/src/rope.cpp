#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
        for (int i = 0; i < num_nodes; ++i)
        {
            Vector2D pos = start + (static_cast<float>(i) / (num_nodes - 1)) * (end - start);
            masses.push_back(new Mass(pos, node_mass, false));
        }
        for (int i = 1; i < num_nodes; ++i)
            springs.push_back(new Spring(masses[i - 1], masses[i], k));
        for (auto &i : pinned_nodes)
            masses[i]->pinned = true;
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D dis = s->m1->position - s->m2->position; 
            Vector2D f = -s->k * (dis / dis.norm()) * (dis.norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity;

                // TODO (Part 2): Add global damping
                m->forces += -0.005f * m->velocity;

                Vector2D vt = m->velocity, xt = m->position, at = m->forces / m->mass;
                Vector2D vt_1 = vt + at * delta_t;

                // explict
                // Vector2D xt_1 = xt + vt * delta_t;
                // semi-implicit
                Vector2D xt_1 = xt + vt_1 * delta_t;

                m->velocity = vt_1;
                m->position = xt_1;

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D dis = s->m1->position - s->m2->position; 
            Vector2D f = -s->k * (dis / dis.norm()) * (dis.norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces += -f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;

                m->forces += m->mass * gravity;
                // TODO (Part 3.1): Set the new position of the rope mass

                Vector2D at = m->forces / m->mass;
                m->position = temp_position+(1.0f-0.00005f)*(temp_position-m->last_position)+at*delta_t*delta_t; 
                // m->position = temp_position + (temp_position - m->last_position) + at * delta_t * delta_t; 
                // TODO (Part 4): Add global Verlet damping
                
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
