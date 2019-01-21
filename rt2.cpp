//------------------------------------------------------------------------------
// Purpose: Implements a ray-traced scene similar to Peter Shirley's 
//          "Ray-Tracing in One Weekend", taking a fully data-oriented approach.
//          Ray casts are processed in batches, as are the different types of shading.
//
// Author:  Andrew Willmott, but this version has pieces cribbed from Peter's 
//          code to get comparable results. E.g., the camera class, some material 
//          code, and balls scene setup.
//------------------------------------------------------------------------------

#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <vector>

namespace
{
    //------------------------------------------------------------------------------
    // VL mini
    //------------------------------------------------------------------------------

    struct Vec3f { float x; float y; float z; Vec3f() {}; Vec3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {} };

    inline Vec3f  operator+ (Vec3f  v)          { return { +v.x, +v.y, +v.z }; }
    inline Vec3f  operator- (Vec3f  v)          { return { -v.x, -v.y, -v.z }; }
    inline Vec3f  operator+ (Vec3f  a, Vec3f b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    inline Vec3f  operator- (Vec3f  a, Vec3f b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline Vec3f  operator* (Vec3f  a, Vec3f b) { return { a.x * b.x, a.y * b.y, a.z * b.z }; }
    inline Vec3f  operator* (float  s, Vec3f a) { return { s   * a.x, s   * a.y, s   * a.z }; }
    inline Vec3f  operator* (Vec3f  a, float s) { return { s   * a.x, s   * a.y, s   * a.z }; }
    inline Vec3f  operator/ (Vec3f  a, Vec3f b) { return { a.x / b.x, a.y / b.y, a.z / b.z }; }
    inline Vec3f  operator/ (float  s, Vec3f a) { return { s   / a.x, s   / a.y, s   / a.z }; }
    inline Vec3f  operator/ (Vec3f  a, float s) { return { a.x / s  , a.y / s  , a.z / s   }; }
    inline Vec3f& operator+=(Vec3f& a, Vec3f b) { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
    inline Vec3f& operator-=(Vec3f& a, Vec3f b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
    inline Vec3f& operator*=(Vec3f& a, Vec3f b) { a.x *= b.x; a.y *= b.y; a.z *= b.z; return a; }
    inline Vec3f& operator/=(Vec3f& a, Vec3f b) { a.x /= b.x; a.y /= b.y; a.z /= b.z; return a; }

    inline float sqr      (float s)          { return s * s; }
    inline float dot      (Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float len      (Vec3f v)          { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
    inline float sqrlen   (Vec3f v)          { return v.x * v.x + v.y * v.y + v.z * v.z; }
    inline Vec3f norm     (Vec3f v)          { return (1.0f / len(v)) * v; }
    inline Vec3f cross    (Vec3f a, Vec3f b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }

    struct PD3f // point/direction data type, used either for rays or directed surface points
    {
        Vec3f p;
        Vec3f d;
    };


    //------------------------------------------------------------------------------
    // RNG support
    //------------------------------------------------------------------------------

    struct cLCG
    {
        uint32_t mState = 0x12345678;

        cLCG() {};
        cLCG(uint32_t seed) : mState(seed) {}

        operator uint32_t()
        {
            uint32_t current(mState);
            mState = uint32_t(mState * uint64_t(1103515245) + 12345);
            return current;
        }
    };
    typedef cLCG tRNG;

    inline float ToUF32 (uint32_t u) { return float(1.0 / 0xFFFFFFFF) * u; }
    inline float ToSF32 (uint32_t u) { return float(2.0 / 0xFFFFFFFF) * u - 1.0f; }
    inline Vec3f ToUVec3f(tRNG& rng) { return { ToUF32(rng), ToUF32(rng), ToUF32(rng) }; }
    inline Vec3f ToSVec3f(tRNG& rng) { return { ToSF32(rng), ToSF32(rng), ToSF32(rng) }; }

    inline Vec3f ToSphere(tRNG& rng)
    {
        while (true)
        {
            Vec3f p = ToSVec3f(rng);
            
            if (sqrlen(p) < 1.0f)
                return p;
        }
    }

    inline Vec3f ToDisc(tRNG& rng)
    {
        while (true)
        {
            Vec3f p = Vec3f(ToSF32(rng), ToSF32(rng), 0.0f);

            if (sqrlen(p) < 1.0f)
                return p;
        }
    }

    template<class T> int size_i(const T& container) { return int(container.size()); }
}


//------------------------------------------------------------------------------
// Camera
//------------------------------------------------------------------------------

struct cCamera
{
    float mLensRadius;

    Vec3f mOrigin;
    Vec3f mLu;
    Vec3f mLv;

    Vec3f mTarget;
    Vec3f mTu;
    Vec3f mTv;

    cCamera(Vec3f lookFrom, Vec3f lookAt, Vec3f vup, float fov, float aspect, float aperture, float focalD)
    {
        mLensRadius = aperture / 2.0f;

        float theta = fov * M_PI / 360.0f;
        float halfHeight = tan(theta);
        float halfWidth  = aspect * halfHeight;

        mOrigin = lookFrom;

        Vec3f lw = norm(lookFrom - lookAt);
        mLu = norm(cross(vup, lw));
        mLv = cross(lw, mLu);

        mTarget = - halfWidth * focalD * mLu - halfHeight * focalD * mLv - focalD * lw;

        mTu = 2 * halfWidth  * focalD * mLu;
        mTv = 2 * halfHeight * focalD * mLv;
    }

    PD3f Ray(tRNG& rng, float s, float t) const    // generate a world-space ray for the given image point
    {
        Vec3f rd = mLensRadius * ToDisc(rng);
        Vec3f offset = mLu * rd.x + mLv * rd.y;

        return { mOrigin + offset, mTarget + s * mTu + t * mTv - offset };
    }
};


//------------------------------------------------------------------------------
// Material support
//------------------------------------------------------------------------------

namespace
{
    // Utilities
    Vec3f Reflect(const Vec3f& v, const Vec3f& n)
    {
        return v - 2 * dot(v, n) * n;
    }

    inline bool Refract(float kn, const Vec3f& d, const Vec3f& n, Vec3f& refractDir)
    {
        Vec3f v = -norm(d);
        float cosNV = dot(n, v);
        
        float cos2 = 1.0f - sqr(kn) * (1.0f - sqr(cosNV));
        
        if (cos2 <= 0.0f)
            return false;   // Total internal reflection

        refractDir = v * -kn + n * (kn * cosNV - sqrt(cos2));
        return true;
    }

    inline float Schlick(float c, float ior)
    {
        float r0 = (1.0f - ior) / (1.0f + ior);
        r0 = r0 * r0;
        
        float ic  = (1.0f - c);
        float ic2 = ic * ic;
        float ic4 = ic2 * ic2;
        float ic5 = ic  * ic4;

        return r0 + (1 - r0) * ic5;
    }
}

enum tMaterialKind
{
    kMaterialDiffuse,
    kMaterialMetal,
    kMaterialGlass,
    kNumMaterials
};

// Diffuse (Lambert)
struct cDiffuseInfo
{
    Vec3f mAlbedo = { 1, 1, 1 };
};

bool Scatter(tRNG& rng, const cDiffuseInfo& info, const PD3f& , const PD3f& hit, Vec3f& attenuation, PD3f& scattered)
{
    Vec3f target = hit.p + hit.d + ToSphere(rng);
    scattered = PD3f { hit.p, target - hit.p };
    attenuation = info.mAlbedo;
    return true;
}

struct cMetalInfo
{
    Vec3f mAlbedo = { 1, 1, 1 };
    float mFuzz   = 1.0f;
};

bool Scatter(tRNG& rng, const cMetalInfo& info, const PD3f& ray, const PD3f& hit, Vec3f& attenuation, PD3f& scattered)
{
    Vec3f reflected = Reflect(norm(ray.d), hit.d);
    scattered = { hit.p, reflected + info.mFuzz * ToSphere(rng) };
    attenuation = info.mAlbedo;
    return (dot(scattered.d, hit.d) > 0);
}

struct cGlassInfo
{
    float mIoR = 1.0f;
};

bool Scatter(tRNG& rng, const cGlassInfo& info, const PD3f& ray, const PD3f& hit, Vec3f& attenuation, PD3f& scattered)
{
    attenuation = Vec3f(1.0f, 1.0f, 1.0f);

    Vec3f outwardNormal;
    float kn;
    float cosine;

    if (dot(ray.d, hit.d) > 0.0f)
    {
        outwardNormal = -hit.d;
        kn = info.mIoR;
        cosine = dot(ray.d, hit.d) / len(ray.d);
        cosine = sqrtf(1 - info.mIoR * info.mIoR * (1 - cosine * cosine));
    }
    else
    {
        outwardNormal = hit.d;
        kn = 1.0f / info.mIoR;
        cosine = -dot(ray.d, hit.d) / len(ray.d);
    }

    Vec3f refracted;
    float reflectionChance;

    if (Refract(kn, ray.d, outwardNormal, refracted))
        reflectionChance = Schlick(cosine, info.mIoR);  // reflect based on Fresnel strength
    else
        reflectionChance = 1.0f;    // total internal reflection, so always reflect

    if (ToUF32(rng) <= reflectionChance)
    {
        Vec3f reflected = Reflect(ray.d, hit.d);
        scattered = { hit.p, reflected };
    }
    else
        scattered = { hit.p, refracted };
    
    return true;
}


//------------------------------------------------------------------------------
// Scene description + intersection
//------------------------------------------------------------------------------

struct cSphere
{
    Vec3f p;
    float r;
};

struct cMaterialDesc
{
    tMaterialKind mKind;
    int           mIndex;
};

struct cScene
{
    std::vector<cSphere>        mSpheres;           // core sphere info
    std::vector<cMaterialDesc>  mSphereMaterials;   // these are references to m*Material below

    std::vector<cDiffuseInfo>   mDiffuseMaterials;
    std::vector<cMetalInfo>     mMetalMaterials;
    std::vector<cGlassInfo>     mGlassMaterials;
};

inline bool Intersect(const PD3f& ray, const cSphere& sphere, float tMin, float tMax, float* tOut)
{
    Vec3f delta = ray.p - sphere.p;

    float a = sqrlen(ray.d);
    float b = dot(ray.d, delta);
    float c = sqrlen(delta) - sqr(sphere.r);
    float d = b * b - a * c;

    if (d <= 0.0f)
        return false;

    float rd = sqrtf(d);

    float t0 = -b - rd;
    float atMax = a * tMax;

    if (t0 >= atMax)
        return false;

    float atMin = a * tMin;

    if (t0 > atMin)
    {
        *tOut = t0 / a;
        return true;
    }

    float t1 = -b + rd;

    if (t1 > atMin && t1 < atMax)
    {
        *tOut = t1 / a;
        return true;
    }

    return false;
}

int Intersect(const cScene& scene, const PD3f& ray, PD3f& out)
{
    float tMin = 0.001f;
    float tMax = FLT_MAX;
    int hitIndex = -1;
    
    for (const cSphere& sphere : scene.mSpheres)
        if (Intersect(ray, sphere, tMin, tMax, &tMax))
            hitIndex = int(&sphere - &scene.mSpheres.front());

    if (hitIndex >= 0)
    {
        out.p = ray.p + tMax * ray.d;
        out.d = norm(out.p - scene.mSpheres[hitIndex].p);
    }
    
    return hitIndex;
}

inline Vec3f SkyColour(const PD3f& r)
{
    Vec3f unitDir = norm(r.d);

    float t = 0.5f * (unitDir.y + 1.0f);
    return Vec3f((1.0f - t), (1.0f - t), (1.0f - t)) + t * Vec3f(0.5f, 0.7f, 1.0f);
}


//------------------------------------------------------------------------------
// Data-oriented or "batch" ray tracer
//------------------------------------------------------------------------------

struct cRayDesc
{
    PD3f    mRay;       // ray we're casting
    Vec3f   mWeight;    // weight we should give the results
    int     mPixel;     // destination pixel to accumulate into
};

struct cHitDesc   // info needed to shade a hit
{
    cRayDesc mRay;
    PD3f     mHit;
    int      mIndex;
};

struct cWorkingSet
{
    std::vector<cRayDesc> mRays;
    std::vector<cHitDesc> mHits[kNumMaterials];
};

bool CastRays(tRNG& rng, const cScene& scene, cWorkingSet* wd, Vec3f image[])
{
//    fprintf(stderr, "Casting %d rays\n", size_i(wd->mRays));

    // Cast the rays and store the hits
    for (int i = 0; i < kNumMaterials; i++)
        wd->mHits[i].clear();

    PD3f hit;

    for (const cRayDesc& ray : wd->mRays)
    {
        int hitIndex = Intersect(scene, ray.mRay, hit);
        
        if (hitIndex >= 0)
        {
            const cMaterialDesc& md = scene.mSphereMaterials[hitIndex];
            wd->mHits[md.mKind].push_back( { ray, hit, md.mIndex } );
        } 
        else
            image[ray.mPixel] += SkyColour(ray.mRay) * ray.mWeight;
    }

    // Deal with the hits, generating more rays
    PD3f scattered;
    Vec3f attenuation;

    wd->mRays.clear();

    // Process each material type separately.
    for (const cHitDesc& sd : wd->mHits[kMaterialDiffuse])
        if (Scatter(rng, scene.mDiffuseMaterials[sd.mIndex], sd.mRay.mRay, sd.mHit, attenuation, scattered))
            wd->mRays.push_back( { scattered, attenuation * sd.mRay.mWeight, sd.mRay.mPixel } );

    for (const cHitDesc& sd : wd->mHits[kMaterialMetal]) 
        if (Scatter(rng, scene.mMetalMaterials[sd.mIndex], sd.mRay.mRay, sd.mHit, attenuation, scattered))
            wd->mRays.push_back( { scattered, attenuation * sd.mRay.mWeight, sd.mRay.mPixel } );

    for (const cHitDesc& sd : wd->mHits[kMaterialGlass]) 
        if (Scatter(rng, scene.mGlassMaterials[sd.mIndex], sd.mRay.mRay, sd.mHit, attenuation, scattered))
            wd->mRays.push_back( { scattered, attenuation * sd.mRay.mWeight, sd.mRay.mPixel } );

    return !wd->mRays.empty();
}

void TraceScene(tRNG& rng, int nx, int ny, int ns, int nd, const cCamera& camera, cScene& scene)
{
    Vec3f* image = new Vec3f[nx * ny]();

    float ws = 1.0f / ns;
    Vec3f w = { ws, ws, ws };   // initial ray weight

    cWorkingSet wd;
    wd.mRays.reserve(nx * ny * ns);
    
    // Queue up base rays
    for (int j = 0; j < ny; j++)
        for (int i = 0; i < nx; i++)
            for (int s = 0; s < ns; s++)
            {
                float u = float(i + ToUF32(rng)) / float(nx);
                float v = float(j + ToUF32(rng)) / float(ny);

                PD3f r = camera.Ray(rng, u, v);

                wd.mRays.push_back( { r, w, i + j * nx } );  
            }

    for (int i = 0; i < nd; i++)
        if (!CastRays(rng, scene, &wd, image))
            break;

//    fprintf(stderr, "Discarding final %d rays\n", size_i(wd.mRays));

    // Now we're done, write the final image
    printf("P3\n%d %d\n255\n", nx, ny);

    for (int j = ny - 1; j >= 0; j--)
        for (int i = 0; i < nx; i++)
        {
            Vec3f c = image[i + j * nx];
            c = Vec3f(sqrtf(c.x), sqrtf(c.y), sqrtf(c.z));  // rough gamma

            int ir = int(255 * c.x);
            int ig = int(255 * c.y);
            int ib = int(255 * c.z);

            printf("%d %d %d\n", ir, ig, ib);
        }
}


//------------------------------------------------------------------------------
// RTiOW Scene
//------------------------------------------------------------------------------

// Helpers
void AddSphere(cScene* scene, Vec3f p, float r)
{
    scene->mSpheres.push_back( { p, r } );
}
void AddDiffuse(cScene* scene, Vec3f albedo)
{
    scene->mSphereMaterials .push_back( { kMaterialDiffuse, size_i(scene->mDiffuseMaterials) } );
    scene->mDiffuseMaterials.push_back( { albedo } );
}
void AddMetal(cScene* scene, Vec3f albedo, float fuzz)
{
    scene->mSphereMaterials.push_back( { kMaterialMetal, size_i(scene->mMetalMaterials) } );
    scene->mMetalMaterials .push_back( { albedo, fuzz } );
}
void AddGlass(cScene* scene, float ior)
{
    scene->mSphereMaterials.push_back( { kMaterialGlass, size_i(scene->mGlassMaterials) } );
    scene->mGlassMaterials .push_back( { ior } );
}
void AddGlass(cScene* scene, int materialIndex)
{
    scene->mSphereMaterials.push_back( { kMaterialGlass, materialIndex } );
}

// So many balls
void AddBalls(tRNG& rng, cScene* scene, int ballGridSize)
{
    AddSphere(scene, Vec3f(0, -1000, 0), 1000);
    AddDiffuse(scene, Vec3f(0.5f, 0.5f, 0.5f));

    int glassMaterial1 = size_i(scene->mGlassMaterials);    // add this material manually, as we'll be reusing it.
    scene->mGlassMaterials.push_back( { 1.5f } );

    const int ns = ballGridSize;
    
    for (int a = -ns; a < ns; a++)
        for (int b = -ns; b < ns; b++)
        {
            float chooseMaterial = ToUF32(rng);

            Vec3f centre(a, 0.2f, b);
            centre.x += 0.9f * ToUF32(rng);
            centre.z += 0.9f * ToUF32(rng);

            if (len(centre - Vec3f(4, 0.2f, 0)) > 0.9f)
            {
                AddSphere(scene, centre, 0.2f);

                if (chooseMaterial < 0.8f)
                    AddDiffuse(scene, Vec3f(ToUF32(rng) * ToUF32(rng), ToUF32(rng) * ToUF32(rng), ToUF32(rng) * ToUF32(rng)));
                else if (chooseMaterial < 0.95f)
                    AddMetal(scene, Vec3f( 0.5f * (1.0f + ToUF32(rng)), 0.5f * (1.0f + ToUF32(rng)), 0.5f * (1.0f + ToUF32(rng)) ), 0.5f * ToUF32(rng));
                else
                    AddGlass(scene, glassMaterial1);
            }
        }

    AddSphere(scene, Vec3f( 0, 1, 0), 1.0f);
    AddSphere(scene, Vec3f(-4, 1, 0), 1.0f);
    AddSphere(scene, Vec3f(+4, 1, 0), 1.0f);

    AddGlass  (scene, glassMaterial1);
    AddDiffuse(scene, Vec3f(0.4, 0.2, 0.1));
    AddMetal  (scene, Vec3f(0.7f, 0.6f, 0.5f), 0.0f);
    
    assert(scene->mSpheres.size() == scene->mSphereMaterials.size());
}


//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------

int main(int argc, const char* const argv[])
{
    argc--;
    argv++;

    // Parse options: <width> <height> <samples> <depth>
    int w = 640;
    int h = 480;
    int ns = 4;
    int nd = 16;
    
    if (argc > 0)
    {
        w = atoi(argv[0]);
        h = w;
    }
    if (argc > 1)
        h = atoi(argv[1]);
    if (argc > 2)
        ns = atoi(argv[2]);
    if (argc > 3)
        nd = atoi(argv[3]);

    // Set up our scene
    cScene scene;
    tRNG sceneRNG;
    AddBalls(sceneRNG, &scene, 11);

    Vec3f viewPos(13, 2, 3);
    Vec3f lookAt(0, 0, 0);
    float focalDistance = 10.0f;
    float aperture = 0.1f;

    cCamera camera(viewPos, lookAt, Vec3f(0, 1, 0), 20, float(w) / float(h), aperture, focalDistance);

    // Trace
    tRNG traceRNG;
    TraceScene(traceRNG, w, h, ns, nd, camera, scene);

    return 0;
}
