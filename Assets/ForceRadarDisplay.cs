using UnityEngine;

public class ForceRadarDisplay : MonoBehaviour
{
    public USVLidarSensorImpl radarCore;
    public float rayLength = 10f;
    public int rayCount = 36;
    public Color noObstacleColor = Color.blue; // 未检测到障碍物时的颜色
    public Color hasObstacleColor = Color.red; // 检测到障碍物时的颜色
    public float originSize = 0.5f;

    private Material _glMaterial;

    void OnRenderObject()
    {
        if (radarCore == null || radarCore.usvTransform == null)
        {
            Debug.LogWarning("雷达核心组件或无人船Transform未绑定");
            return;
        }

        // 关键修改：使用雷达偏移后的实际位置作为射线原点
        // 不再直接用无人船位置，而是用雷达自身的位置（已包含偏移）
        Vector3 origin = radarCore.transform.position;

        // 初始化材质
        if (_glMaterial == null)
        {
            _glMaterial = new Material(Shader.Find("Hidden/Internal-Colored"));
            _glMaterial.hideFlags = HideFlags.HideAndDontSave;
        }
        _glMaterial.SetPass(0);

        // 开始绘制
        GL.PushMatrix();
        GL.Begin(GL.LINES);

        // 绘制原点十字（固定为白色）
        GL.Color(Color.white);
        DrawCross(origin, originSize);

        // 获取雷达检测数据
        float[] distances = radarCore.GetDistances();
        int sampleCount = distances.Length;
        float maxDistance = radarCore.maxDetectionDistance;

        // 绘制每条射线（根据检测结果动态变色）
        float angleStep = 360f / sampleCount;
        for (int i = 0; i < sampleCount; i++)
        {
            // 计算当前射线角度（弧度）
            float angleRad = (i * angleStep) * Mathf.Deg2Rad;

            // 根据距离判断是否检测到障碍物
            bool hasObstacle = distances[i] < maxDistance - 0.1f; // 留一点误差余量
            GL.Color(hasObstacle ? hasObstacleColor : noObstacleColor);

            // 计算射线终点（检测到障碍物时用实际距离，否则用最大距离）
            float drawDistance = hasObstacle ? distances[i] : maxDistance;
            Vector3 end = new Vector3(
                origin.x + Mathf.Cos(angleRad) * drawDistance,
                origin.y,
                origin.z + Mathf.Sin(angleRad) * drawDistance
            );

            // 绘制射线
            GL.Vertex(origin);
            GL.Vertex(end);
        }

        GL.End();
        GL.PopMatrix();
    }

    private void DrawCross(Vector3 center, float size)
    {
        // 水平线（X轴）
        GL.Vertex(new Vector3(center.x + size, center.y, center.z));
        GL.Vertex(new Vector3(center.x - size, center.y, center.z));
        // 垂直线（Z轴）
        GL.Vertex(new Vector3(center.x, center.y, center.z + size));
        GL.Vertex(new Vector3(center.x, center.y, center.z - size));
    }

    // Gizmos备份绘制（场景视图同步颜色逻辑）
    void OnDrawGizmos()
    {
        if (radarCore == null || radarCore.usvTransform == null) return;

        // 关键修改：Gizmos也使用雷达偏移后的位置
        Vector3 origin = radarCore.transform.position;
        Gizmos.color = Color.white;
        Gizmos.DrawSphere(origin, originSize);

        float[] distances = radarCore.GetDistances();
        int sampleCount = distances.Length;
        float maxDistance = radarCore.maxDetectionDistance;
        float angleStep = 360f / sampleCount;

        for (int i = 0; i < sampleCount; i++)
        {
            bool hasObstacle = distances[i] < maxDistance - 0.1f;
            Gizmos.color = hasObstacle ? hasObstacleColor : noObstacleColor;

            float angle = i * angleStep;
            // 射线方向使用雷达的朝向（已包含旋转偏移）
            Vector3 direction = Quaternion.Euler(0, angle, 0) * radarCore.transform.forward;
            float drawDistance = hasObstacle ? distances[i] : maxDistance;
            Gizmos.DrawLine(origin, origin + direction * drawDistance);
        }
    }
}