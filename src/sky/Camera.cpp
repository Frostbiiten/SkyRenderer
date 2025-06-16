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
        Matrix inv_rot = MatrixInvert(QuaternionToMatrix(rotation));
        Matrix inv_pos = MatrixTranslate(-position.x, -position.y, -position.z);
        return MatrixMultiply(inv_pos, inv_rot);
    }

    float Camera::get_near_clip() const
    {
        return near_clip;
    }

    float Camera::get_far_clip() const
    {
        return far_clip;
    }

    Matrix Camera::get_projection_matrix(int screen_width, int screen_height) const
    {
        return MatrixPerspective(fov * DEG2RAD, (float)screen_width / screen_height, near_clip, far_clip);
    }
}