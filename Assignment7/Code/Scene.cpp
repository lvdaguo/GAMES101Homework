//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// shade (p, wo)
//   sampleLight ( inter , pdf_light )
//   Get x, ws , NN , emit from inter
//   Shoot a ray from p to x
//   If the ray is not blocked in the middle
//     L_dir = emit * eval (wo , ws , N) * dot (ws , N) * dot (-ws ,
//     NN) / |x-p |^2 / pdf_light
// 
//   L_indir = 0.0
//   Test Russian Roulette with probability RussianRoulette
//   wi = sample (wo , N)
//   Trace a ray r(p, wi)
//   If ray r hit a non - emitting object at q
//     L_indir = shade (q, wi) * eval (wo , wi , N) * dot (wi , N)
//     / pdf (wo , wi , N) / RussianRoulette
//   Return L_dir + L_indir

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    if (inter.happened == false) return Vector3f();
    Material* m = inter.m;
    if (m->hasEmission()) return m->getEmission();
    // 如果严格按照上述算法实现,你会发现渲染结果中光源区域为纯黑。请分析
    // 这一现象的原因,并且修改 Path Tracing 算法使光源可见。
    // solved

    Vector3f l_dir, l_indir;
    Vector3f p = inter.coords, wo = -ray.direction; 

    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light);

    Vector3f x = inter_light.coords, ws = normalize(x - p), emit = inter_light.emit;
    Vector3f N = inter.normal, NN = inter_light.normal;

    Intersection block_check = intersect(Ray(p, ws));
    // If the ray is not blocked in the middle
    if ((block_check.coords - x).norm() < 0.1f)
    {
        l_dir = emit * m->eval(wo, ws, N) * dotProduct(ws, N) * 
        dotProduct(-ws, NN) / std::pow((x - p).norm(), 2) / pdf_light;
    }
    // Test Russian Roulette with probability RussianRoulette
    if (get_random_float() <= RussianRoulette)
    {
        Vector3f wi = m->sample(wo, N);
        Intersection inter_indir = intersect(Ray(p, wi));
        // If ray r hit a non - emitting object at q
        if (inter_indir.happened && inter_indir.m->hasEmission() == false)
        {  
            l_indir = castRay(Ray(p, wi), depth + 1) * m->eval(wo, wi, N) * 
            dotProduct(wi, N) / m->pdf(wo, wi, N) / RussianRoulette;
        }
    }
    return l_dir + l_indir;
}