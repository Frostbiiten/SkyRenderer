// SKY
#include <sky/Camera.h>

namespace sky
{
    Camera::Camera (float fov, Vector3 position, Quaternion rotation) :
            position { position },
            rotation { rotation },
            fov(fov)
            {}

    Matrix Camera::get_view_matrix() const
    {
        Matrix invRot = MatrixInvert(QuaternionToMatrix(rotation));
        Matrix invPos = MatrixTranslate(-position.x, -position.y, -position.z);
        return MatrixMultiply(invPos, invRot);
    }

    float Camera::get_near_clip() const
    {
        return near_clip;
    }

    float Camera::get_far_clip() const
    {
        return far_clip;
    }

    Matrix Camera::get_projection_matrix(int screenWidth, int screenHeight) const
    {
        return MatrixPerspective(fov * DEG2RAD, (float)screenWidth / screenHeight, near_clip, far_clip);
    }
}