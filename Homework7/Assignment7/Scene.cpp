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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    if (!inter.happened) return Vector3f();
    if(inter.m->hasEmission()) return inter.m->getEmission();

    Vector3f l_dir(0, 0, 0), l_indir(0, 0, 0);
    Intersection lightPos;
    float lightPdf = 0.0f;
    sampleLight(lightPos, lightPdf);
    auto objtolight = lightPos.coords - inter.coords, objtolightDir = objtolight.normalized();
    float dist = objtolight.x * objtolight.x + objtolight.y * objtolight.y + objtolight.z * objtolight.z; 

    Ray objtolightRay(inter.coords, objtolightDir);
    Intersection check = intersect(objtolightRay);
    //if (check.distance - objtolight.norm() > -EPSILON)
    if (check.distance - objtolight.norm() > -0.005)
    {
        l_dir = lightPos.emit * inter.m->eval(ray.direction, objtolightDir, inter.normal) * dotProduct(objtolightDir, inter.normal) * dotProduct(-objtolightDir, lightPos.normal) / dist / lightPdf;
    }
    
    if (get_random_float() > RussianRoulette) return l_dir;

    Vector3f objtoobjDir = inter.m->sample(ray.direction, inter.normal).normalized();
    Ray objtoobjRay(inter.coords, objtoobjDir);
    Intersection objInter = intersect(objtoobjRay);
    if (objInter.happened && !objInter.m->hasEmission())
    {
        float pdf = inter.m->pdf(ray.direction, objtoobjDir, inter.normal);
        l_indir = castRay(objtoobjRay, depth + 1) * inter.m->eval(ray.direction, objtoobjDir, inter.normal) * dotProduct(objtolightDir, inter.normal) / pdf / RussianRoulette;
    }
    return l_dir + l_indir;
}
