using UnityEngine;
public class ForceRadarDisplay : MonoBehaviour
{
    public USVLidarSensorImpl radarCore;
    public float rayLength = 10f;
    public int rayCount = 36;
    public Color noObstacleColor = Color.blue;
    public Color hasObstacleColor = Color.red;
    public float originSize = 0.5f;
    private Material _glMaterial;

    void OnRenderObject()
    {
        if (radarCore == null || radarCore.usvTransform == null)
        {
            Debug.LogWarning("雷达核心或USV Transform未设置");
            return;
        }

        // 计算带偏移的原点位置
        Vector3 origin = radarCore.transform.position;
        // 应用XZ轴显示偏移
        origin.x += radarCore.displayOffsetX;
        origin.z += radarCore.displayOffsetZ;

        float radarActualHeight = radarCore.raycastHeight;

        if (_glMaterial == null)
        {
            _glMaterial = new Material(Shader.Find("Hidden/Internal-Colored"));
            _glMaterial.hideFlags = HideFlags.HideAndDontSave;
        }
        _glMaterial.SetPass(0);

        GL.PushMatrix();
        GL.Begin(GL.LINES);

        // 绘制原点十字（应用所有偏移）
        GL.Color(Color.white);
        DrawCross(origin, originSize, radarActualHeight);

        float[] distances = radarCore.GetDistances();
        int sampleCount = distances.Length;
        float maxDistance = radarCore.maxDetectionDistance;
        float angleStep = 360f / sampleCount;

        for (int i = 0; i < sampleCount; i++)
        {
            float angleRad = (i * angleStep) * Mathf.Deg2Rad;
            bool hasObstacle = distances[i] < maxDistance - 0.1f;
            GL.Color(hasObstacle ? hasObstacleColor : noObstacleColor);

            float drawDistance = hasObstacle ? distances[i] : maxDistance;
            Vector3 end = new Vector3(
                origin.x + Mathf.Cos(angleRad) * drawDistance,  // 使用带偏移的原点计算终点
                radarActualHeight,
                origin.z + Mathf.Sin(angleRad) * drawDistance   // 使用带偏移的原点计算终点
            );

            GL.Vertex(new Vector3(origin.x, radarActualHeight, origin.z));
            GL.Vertex(end);
        }

        GL.End();
        GL.PopMatrix();
    }

    private void DrawCross(Vector3 center, float size, float radarHeight)
    {
        // 水平轴（X轴）
        GL.Vertex(new Vector3(center.x + size, radarHeight, center.z));
        GL.Vertex(new Vector3(center.x - size, radarHeight, center.z));
        // 垂直轴（Z轴）
        GL.Vertex(new Vector3(center.x, radarHeight, center.z + size));
        GL.Vertex(new Vector3(center.x, radarHeight, center.z - size));
    }

    void OnDrawGizmos()
    {
        if (radarCore == null || radarCore.usvTransform == null) return;

        // 计算带偏移的原点位置
        Vector3 origin = radarCore.transform.position;
        origin.x += radarCore.displayOffsetX;
        origin.z += radarCore.displayOffsetZ;

        float radarActualHeight = radarCore.raycastHeight;

        // 绘制原点球体
        Gizmos.color = Color.white;
        Vector3 spherePos = new Vector3(origin.x, radarActualHeight, origin.z);
        Gizmos.DrawSphere(spherePos, originSize);

        float[] distances = radarCore.GetDistances();
        int sampleCount = distances.Length;
        float maxDistance = radarCore.maxDetectionDistance;
        float angleStep = 360f / sampleCount;

        for (int i = 0; i < sampleCount; i++)
        {
            bool hasObstacle = distances[i] < maxDistance - 0.1f;
            Gizmos.color = hasObstacle ? hasObstacleColor : noObstacleColor;

            float angle = i * angleStep;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * radarCore.transform.forward;
            float drawDistance = hasObstacle ? distances[i] : maxDistance;

            // 计算带偏移的终点位置
            Vector3 gizmoEnd = origin + direction * drawDistance;
            gizmoEnd.y = radarActualHeight;

            Gizmos.DrawLine(spherePos, gizmoEnd);
        }
    }
}