using UnityEngine;
using System;
using System.Collections.Generic;

#if UNITY_EDITOR
using UnityEditor;
#endif

// ============================================================
// 确保所有代码都在命名空间内
// ============================================================
namespace USVGridSystem
{
    // 节点结构体
    internal struct Node
    {
        public bool walkable;
        public Vector3 worldPosition;
        public int gridX;
        public int gridY;

        public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY)
        {
            walkable = _walkable;
            worldPosition = _worldPos;
            gridX = _gridX;
            gridY = _gridY;
        }
    }

    // GridManager 类
    public class GridManager : MonoBehaviour
    {
        [Header("水域挂载配置（二选一）")]
        [Tooltip("是否自动读取水域实际尺寸（优先Collider，再读Mesh）")]
        public bool autoReadWaterSize = true;
        [Tooltip("手动输入水域X轴长度（米），自动读取关闭时生效")]
        public float manualWaterSizeX = 100f;
        [Tooltip("手动输入水域Z轴长度（米），自动读取关闭时生效")]
        public float manualWaterSizeZ = 80f;

        [Header("栅格基础配置")]
        public float 栅格尺寸 = 1f;
        public Transform 水域平面;
        public LayerMask obstacleLayer;
        public Vector3 栅格原点;
        public int 栅格宽度;
        public int 栅格高度;

        [Header("初始化性能优化")]
        public int 每帧初始化数量 = 50;

        [Header("障碍物检测配置")]
        public float obstacleCheckRadius = 0.5f;
        public float radiusOffset = 0f;

        [Header("Gizmos显示设置")]
        public float 栅格线高度 = 0.1f;
        public float 障碍物显示高度 = 0.2f;
        public Color 栅格线颜色 = new Color(0.5f, 0.5f, 0.5f, 0.1f);
        public Color 障碍物颜色 = new Color(1f, 0f, 0f, 0.7f);

        [Header("初始化超时保护")]
        public float initTimeout = 10f;

        public float gridCellSize = 1f;
        public Vector3 gridOrigin = Vector3.zero;

        private Node[,] 栅格地图;
        private bool isGridReady = false;
        private Collider[] 碰撞检测结果 = new Collider[1];
        private Vector2 水域大小缓存;
        private float 栅格半尺寸;

        private float waterMinX;
        private float waterMaxX;
        private float waterMinZ;
        private float waterMaxZ;
        public float WaterHeight => 水域平面.position.y;
        public float WaterMinY { get; private set; }
        public float WaterMaxY { get; private set; }

        private float lastWaterMinX;
        private float lastWaterMaxX;
        private float lastWaterMinZ;
        private float lastWaterMaxZ;
        private float lastWaterMinY;
        private float lastWaterMaxY;

        private static GridManager _instance;

        public bool IsInitialized { get; private set; }

        // ========== 公共属性（供其他组件调用） ==========
        public float WaterMinX => waterMinX;
        public float WaterMaxX => waterMaxX;
        public float WaterMinZ => waterMinZ;
        public float WaterMaxZ => waterMaxZ;

        public Vector3 DefaultStartPosition
        {
            get
            {
                if (!IsInitialized)
                {
                    Debug.LogWarning("GridManager：默认起始位置获取失败，栅格未初始化！");
                    return Vector3.zero;
                }
                return 水域平面.position + new Vector3(0, 0.05f, 0);
            }
        }

        public int gridWidth => 栅格宽度;
        public int gridHeight => 栅格高度;

        void Start()
        {
            if (水域平面 == null)
            {
                Debug.LogError("GridManager：未赋值水域平面！");
                IsInitialized = false;
                return;
            }

            栅格半尺寸 = 栅格尺寸 / 2f;
            计算水域大小();

            栅格宽度 = Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸);
            栅格高度 = Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸);

            栅格宽度 = Mathf.Clamp(栅格宽度, 10, Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸));
            栅格高度 = Mathf.Clamp(栅格高度, 10, Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸));

            栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
            栅格原点.y = 水域平面.position.y;
            gridOrigin = 栅格原点;
            gridCellSize = 栅格尺寸;

            栅格地图 = new Node[栅格宽度, 栅格高度];

            同步初始化栅格();
            标记障碍物();

            isGridReady = true;
            IsInitialized = true;

            Debug.Log($"GridManager：核心栅格同步初始化完成，水域尺寸：{水域大小缓存.x}x{水域大小缓存.y}，栅格参数：{栅格宽度}x{栅格高度}");
        }

        void 同步初始化栅格()
        {
            for (int x = 0; x < 栅格宽度; x++)
            {
                for (int z = 0; z < 栅格高度; z++)
                {
                    Vector3 节点位置 = 栅格原点 + new Vector3(
                        x * 栅格尺寸 + 栅格半尺寸,
                        水域平面.position.y,
                        z * 栅格尺寸 + 栅格半尺寸
                    );

                    栅格地图[x, z] = new Node(true, 节点位置, x, z);
                }
            }
        }

        public bool IsValidGridPosition(Vector2Int gridPos)
        {
            return gridPos.x >= 0 && gridPos.x < 栅格宽度 && gridPos.y >= 0 && gridPos.y < 栅格高度;
        }

        public bool IsGridReady()
        {
            return isGridReady && IsInitialized;
        }

        public void 标记障碍物(Camera 主相机 = null)
        {
            if (栅格地图 == null)
            {
                Debug.LogWarning("GridManager：标记障碍物失败，栅格地图未初始化！");
                return;
            }
            float 实际检测半径 = 栅格半尺寸 + obstacleCheckRadius + radiusOffset;
            int 总节点数 = 栅格宽度 * 栅格高度;
            int 障碍物数量 = 0;

            Array.Clear(碰撞检测结果, 0, 碰撞检测结果.Length);

            for (int i = 0; i < 总节点数; i += 8)
            {
                for (int j = 0; j < 8 && (i + j) < 总节点数; j++)
                {
                    int 索引 = i + j;
                    int x = 索引 / 栅格高度;
                    int z = 索引 % 栅格高度;
                    Node 节点 = 栅格地图[x, z];

                    int 碰撞数量 = Physics.OverlapSphereNonAlloc(
                        节点.worldPosition,
                        实际检测半径,
                        碰撞检测结果,
                        obstacleLayer,
                        QueryTriggerInteraction.Ignore
                    );

                    if (碰撞数量 > 0)
                    {
                        节点.walkable = false;
                        障碍物数量++;
                    }
                    else
                    {
                        节点.walkable = true;
                    }
                    栅格地图[x, z] = 节点;
                }
            }
            Debug.Log($"GridManager：障碍物标记完成，共检测{总节点数}个节点，发现{障碍物数量}个障碍物节点");
        }

        public void 重置栅格()
        {
            重新初始化栅格数据();
            标记障碍物();
            IsInitialized = true;
            Debug.Log("GridManager：栅格已重置并重新标记障碍物");
        }

        public void 初始化栅格()
        {
            重新初始化栅格数据();
            标记障碍物();
            IsInitialized = true;
            Debug.Log("GridManager：栅格初始化完成（适配SpawnManager调用）");
        }

        private void 重新初始化栅格数据()
        {
            if (水域平面 == null)
            {
                Debug.LogError("GridManager：重新初始化失败，未赋值水域平面！");
                IsInitialized = false;
                return;
            }

            计算水域大小();
            int 新宽度 = Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸);
            int 新高度 = Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸);

            新宽度 = Mathf.Clamp(新宽度, 10, Mathf.RoundToInt(水域大小缓存.x / 栅格尺寸));
            新高度 = Mathf.Clamp(新高度, 10, Mathf.RoundToInt(水域大小缓存.y / 栅格尺寸));

            if (栅格地图 == null || 栅格地图.GetLength(0) != 新宽度 || 栅格地图.GetLength(1) != 新高度)
            {
                栅格地图 = new Node[新宽度, 新高度];
            }

            栅格宽度 = 新宽度;
            栅格高度 = 新高度;
            栅格原点 = 水域平面.position - new Vector3(水域大小缓存.x / 2, 0, 水域大小缓存.y / 2);
            栅格原点.y = 水域平面.position.y;
            gridOrigin = 栅格原点;
            gridCellSize = 栅格尺寸;
            栅格半尺寸 = 栅格尺寸 / 2f;

            int 总节点数 = 栅格宽度 * 栅格高度;
            for (int i = 0; i < 总节点数; i++)
            {
                int x = i / 栅格高度;
                int z = i % 栅格高度;
                Vector3 节点位置 = 栅格原点 + new Vector3(
                    x * 栅格尺寸 + 栅格半尺寸,
                    水域平面.position.y,
                    z * 栅格尺寸 + 栅格半尺寸
                );
                栅格地图[x, z] = new Node(true, 节点位置, x, z);
            }

            isGridReady = true;
            Debug.Log($"GridManager：重新初始化完成，水域尺寸：{水域大小缓存.x}x{水域大小缓存.y}，栅格参数：{栅格宽度}x{栅格高度}");
        }

        private void 计算水域大小()
        {
            if (水域平面 == null)
            {
                水域大小缓存 = Vector2.zero;
                Debug.LogError("GridManager：计算水域大小失败，未赋值水域平面！");
                return;
            }

            Vector2 newWaterSize;
            if (autoReadWaterSize)
            {
                newWaterSize = GetAutoWaterSize(水域平面.gameObject);
            }
            else
            {
                newWaterSize = new Vector2(manualWaterSizeX, manualWaterSizeZ);
            }

            if (Mathf.Abs(newWaterSize.x - 水域大小缓存.x) > 0.01f || Mathf.Abs(newWaterSize.y - 水域大小缓存.y) > 0.01f)
            {
                水域大小缓存 = newWaterSize;
            }

            waterMinX = 水域平面.position.x - 水域大小缓存.x / 2;
            waterMaxX = 水域平面.position.x + 水域大小缓存.x / 2;
            waterMinZ = 水域平面.position.z - 水域大小缓存.y / 2;
            waterMaxZ = 水域平面.position.z + 水域大小缓存.y / 2;
            WaterMinY = 水域平面.position.y - 0.1f;
            WaterMaxY = 水域平面.position.y + 0.1f;

            if (Mathf.Abs(waterMinX - lastWaterMinX) > 0.01f ||
                Mathf.Abs(waterMaxX - lastWaterMaxX) > 0.01f ||
                Mathf.Abs(waterMinZ - lastWaterMinZ) > 0.01f ||
                Mathf.Abs(waterMaxZ - lastWaterMaxZ) > 0.01f ||
                Mathf.Abs(WaterMinY - lastWaterMinY) > 0.01f ||
                Mathf.Abs(WaterMaxY - lastWaterMaxY) > 0.01f)
            {
                Debug.Log($"GridManager：水域边界计算完成 → X[{waterMinX:F1},{waterMaxX:F1}] Z[{waterMinZ:F1},{waterMaxZ:F1}] Y[{WaterMinY:F1},{WaterMaxY:F1}]");

                lastWaterMinX = waterMinX;
                lastWaterMaxX = waterMaxX;
                lastWaterMinZ = waterMinZ;
                lastWaterMaxZ = waterMaxZ;
                lastWaterMinY = WaterMinY;
                lastWaterMaxY = WaterMaxY;
            }
        }

        private Vector2 GetAutoWaterSize(GameObject waterObj)
        {
            Collider waterCollider = waterObj.GetComponent<Collider>();
            if (waterCollider != null)
            {
                return new Vector2(
                    Mathf.Abs(waterCollider.bounds.size.x),
                    Mathf.Abs(waterCollider.bounds.size.z)
                );
            }

            MeshFilter waterMesh = waterObj.GetComponent<MeshFilter>();
            if (waterMesh != null && waterMesh.mesh != null)
            {
                return new Vector2(
                    Mathf.Abs(waterMesh.mesh.bounds.size.x * waterObj.transform.lossyScale.x),
                    Mathf.Abs(waterMesh.mesh.bounds.size.z * waterObj.transform.lossyScale.z)
                );
            }

            Debug.LogWarning("GridManager：自动读取水域尺寸失败，使用手动输入默认值");
            return new Vector2(manualWaterSizeX, manualWaterSizeZ);
        }

        public Vector2Int 世界转栅格(Vector3 世界坐标)
        {
            if (!isGridReady)
            {
                Debug.LogWarning("GridManager：世界转栅格失败，栅格未初始化完成！");
                return Vector2Int.zero;
            }

            Vector3 偏移 = 世界坐标 - 栅格原点;
            int x = Mathf.FloorToInt(偏移.x / 栅格尺寸);
            int z = Mathf.FloorToInt(偏移.z / 栅格尺寸);
            x = Mathf.Clamp(x, 0, 栅格宽度 - 1);
            z = Mathf.Clamp(z, 0, 栅格高度 - 1);
            return new Vector2Int(x, z);
        }

        public Vector3 栅格转世界(Vector2Int 栅格坐标)
        {
            if (!isGridReady)
            {
                Debug.LogWarning("GridManager：栅格转世界失败，栅格未初始化完成！");
                return Vector3.zero;
            }

            int x = Mathf.Clamp(栅格坐标.x, 0, 栅格宽度 - 1);
            int z = Mathf.Clamp(栅格坐标.y, 0, 栅格高度 - 1);
            return 栅格原点 + new Vector3(
                x * 栅格尺寸 + 栅格半尺寸,
                水域平面.position.y,
                z * 栅格尺寸 + 栅格半尺寸
            );
        }

        public Vector2Int WorldToGrid(Vector3 worldPos)
        {
            return 世界转栅格(worldPos);
        }

        public Vector3 GridToWorld(Vector2Int gridPos)
        {
            return 栅格转世界(gridPos);
        }

        public bool IsGridPassable(Vector2Int gridPos)
        {
            return 栅格是否可通行(gridPos);
        }

        public bool 栅格是否可通行(Vector2Int 栅格坐标)
        {
            if (!isGridReady || 栅格地图 == null)
            {
                Debug.LogWarning("GridManager：检查栅格可通行性失败，栅格未初始化完成！");
                return false;
            }
            if (!IsValidGridPosition(栅格坐标))
            {
                Debug.LogWarning($"GridManager：栅格坐标{栅格坐标}无效，不可通行！");
                return false;
            }
            return 栅格地图[栅格坐标.x, 栅格坐标.y].walkable;
        }

        public static GridManager Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = UnityEngine.Object.FindAnyObjectByType<GridManager>();
                    if (_instance == null)
                    {
                        GameObject go = new GameObject("GridManager");
                        _instance = go.AddComponent<GridManager>();
                    }
                }
                return _instance;
            }
        }

        public bool IsWalkable(Vector2Int gridPos)
        {
            return 栅格是否可通行(gridPos);
        }

        public Vector3 GetRandomWalkablePosition()
        {
            if (!IsInitialized)
            {
                Debug.LogError("GridManager：获取随机通行点失败，栅格未初始化完成！");
                return Vector3.zero;
            }

            List<Vector2Int> walkableGrids = new List<Vector2Int>();

            for (int x = 0; x < 栅格宽度; x++)
            {
                for (int y = 0; y < 栅格高度; y++)
                {
                    Vector2Int gridPos = new Vector2Int(x, y);
                    if (IsWalkable(gridPos))
                    {
                        walkableGrids.Add(gridPos);
                    }
                }
            }

            if (walkableGrids.Count == 0)
            {
                Debug.LogError("GridManager：无可用的通行栅格！");
                return Vector3.zero;
            }

            Vector2Int randomGrid = walkableGrids[UnityEngine.Random.Range(0, walkableGrids.Count)];
            Vector3 worldPos = GridToWorld(randomGrid);
            worldPos.y = 0.05f;
            return worldPos;
        }

        public List<Vector3> GetAllSafePositions()
        {
            if (!IsInitialized)
            {
                Debug.LogError("GridManager：获取安全位置失败，栅格未初始化完成！");
                return new List<Vector3>();
            }

            List<Vector3> safePositions = new List<Vector3>();
            for (int x = 0; x < 栅格宽度; x++)
            {
                for (int y = 0; y < 栅格高度; y++)
                {
                    Vector2Int gridPos = new Vector2Int(x, y);
                    if (IsWalkable(gridPos))
                    {
                        Vector3 worldPos = GridToWorld(gridPos);
                        worldPos.y = 0.05f;
                        safePositions.Add(worldPos);
                    }
                }
            }
            return safePositions;
        }

        [ContextMenu("强制刷新栅格和障碍物")]
        public void 强制刷新栅格()
        {
            重新初始化栅格数据();
            标记障碍物();
            IsInitialized = true;
            Debug.Log("GridManager：已强制刷新栅格和障碍物标记");
        }

        [ContextMenu("定位到栅格原点")]
        public void 定位到栅格原点()
        {
#if UNITY_EDITOR
            if (SceneView.lastActiveSceneView == null)
            {
                Debug.LogWarning("GridManager：未找到SceneView，无法定位");
                return;
            }

            Bounds 栅格范围 = new Bounds(
                栅格原点 + new Vector3(栅格宽度 * 栅格尺寸 / 2, 0, 栅格高度 * 栅格尺寸 / 2),
                new Vector3(栅格宽度 * 栅格尺寸, 10, 栅格高度 * 栅格尺寸)
            );
            SceneView.lastActiveSceneView.Frame(栅格范围);
            Debug.Log($"GridManager：已定位到栅格中心，坐标：{栅格范围.center}");
#endif
        }

        private void OnDrawGizmos()
        {
            if (水域平面 == null)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireCube(Vector3.zero, new Vector3(10, 0.1f, 10));
                return;
            }

            if (Application.isPlaying || 水域平面 == null)
            {
                计算水域大小();
            }

            Gizmos.color = new Color(0.3f, 0.3f, 0.3f, 0.1f);
            Gizmos.DrawWireCube(水域平面.position, new Vector3(水域大小缓存.x, 0.1f, 水域大小缓存.y));

            if (!isGridReady || 栅格地图 == null) return;

            Gizmos.color = 栅格线颜色;
            float gridMaxX = 栅格原点.x + 栅格宽度 * 栅格尺寸;
            float gridMaxZ = 栅格原点.z + 栅格高度 * 栅格尺寸;
            float waterMinX = 水域平面.position.x - 水域大小缓存.x / 2;
            float waterMaxX = 水域平面.position.x + 水域大小缓存.x / 2;
            float waterMinZ = 水域平面.position.z - 水域大小缓存.y / 2;
            float waterMaxZ = 水域平面.position.z + 水域大小缓存.y / 2;

            for (int x = 0; x <= 栅格宽度; x++)
            {
                float lineX = 栅格原点.x + x * 栅格尺寸;
                if (lineX >= waterMinX && lineX <= waterMaxX)
                {
                    Vector3 起点 = new Vector3(lineX, 栅格线高度, Mathf.Max(栅格原点.z, waterMinZ));
                    Vector3 终点 = new Vector3(lineX, 栅格线高度, Mathf.Min(gridMaxZ, waterMaxZ));
                    Gizmos.DrawLine(起点, 终点);
                }
            }

            for (int z = 0; z <= 栅格高度; z++)
            {
                float lineZ = 栅格原点.z + z * 栅格尺寸;
                if (lineZ >= waterMinZ && lineZ <= waterMaxZ)
                {
                    Vector3 起点 = new Vector3(Mathf.Max(栅格原点.x, waterMinX), 栅格线高度, lineZ);
                    Vector3 终点 = new Vector3(Mathf.Min(gridMaxX, waterMaxX), 栅格线高度, lineZ);
                    Gizmos.DrawLine(起点, 终点);
                }
            }

            Gizmos.color = 障碍物颜色;
            for (int x = 0; x < 栅格宽度; x++)
            {
                for (int z = 0; z < 栅格高度; z++)
                {
                    bool isReallyObstacle = !栅格地图[x, z].walkable;
                    Collider[] tempColliders = Physics.OverlapSphere(
                        栅格地图[x, z].worldPosition,
                        栅格半尺寸 + obstacleCheckRadius,
                        obstacleLayer
                    );
                    isReallyObstacle = tempColliders.Length > 0;

                    if (isReallyObstacle)
                    {
                        Vector3 障碍物中心 = 栅格转世界(new Vector2Int(x, z));
                        if (障碍物中心.x >= waterMinX && 障碍物中心.x <= waterMaxX &&
                            障碍物中心.z >= waterMinZ && 障碍物中心.z <= waterMaxZ)
                        {
                            障碍物中心.y = 障碍物显示高度;
                            Gizmos.DrawCube(障碍物中心, new Vector3(栅格尺寸 * 0.8f, 0.2f, 栅格尺寸 * 0.8f));
                        }
                    }
                }
            }

            Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.05f);
            Gizmos.DrawWireCube(
                栅格原点 + new Vector3(栅格宽度 * 栅格尺寸 / 2, 栅格线高度, 栅格高度 * 栅格尺寸 / 2),
                new Vector3(栅格宽度 * 栅格尺寸, 0.1f, 栅格高度 * 栅格尺寸)
            );

            // 绘制路径 - 黄(已走) / 绿(未走) / 紫(终点)
            ImprovedAStar pathfinder = UnityEngine.Object.FindAnyObjectByType<ImprovedAStar>();
            if (pathfinder != null && pathfinder.path != null && pathfinder.path.Count > 1)
            {
                USV_LocalPlanner localPlanner = UnityEngine.Object.FindAnyObjectByType<USV_LocalPlanner>();
                Vector3 currentPos = localPlanner != null ? localPlanner.transform.position : Vector3.zero;

                int nearestIndex = 0;
                float minDist = float.MaxValue;
                for (int i = 0; i < pathfinder.path.Count; i++)
                {
                    Vector3 worldPos = 栅格转世界(pathfinder.path[i]);
                    float dist = Vector3.Distance(new Vector3(currentPos.x, 0, currentPos.z),
                                                   new Vector3(worldPos.x, 0, worldPos.z));
                    if (dist < minDist)
                    {
                        minDist = dist;
                        nearestIndex = i;
                    }
                }
                nearestIndex = Mathf.Clamp(nearestIndex, 0, pathfinder.path.Count - 2);

                for (int i = 0; i < pathfinder.path.Count - 1; i++)
                {
                    Vector3 start = 栅格转世界(pathfinder.path[i]);
                    Vector3 end = 栅格转世界(pathfinder.path[i + 1]);

                    if (start.x >= waterMinX && start.x <= waterMaxX && start.z >= waterMinZ && start.z <= waterMaxZ &&
                        end.x >= waterMinX && end.x <= waterMaxX && end.z >= waterMinZ && end.z <= waterMaxZ)
                    {
                        start.y = 0.1f;
                        end.y = 0.1f;

                        if (i < nearestIndex)
                        {
                            Gizmos.color = Color.green;
                        }
                        else
                        {
                            Gizmos.color = Color.yellow;
                        }
                        Gizmos.DrawLine(start, end);
                    }
                }

                // 绘制终点 (紫色)
                Vector3 finalPoint = 栅格转世界(pathfinder.path[pathfinder.path.Count - 1]);
                if (finalPoint.x >= waterMinX && finalPoint.x <= waterMaxX && finalPoint.z >= waterMinZ && finalPoint.z <= waterMaxZ)
                {
                    finalPoint.y = 0.1f;
                    Gizmos.color = new Color(0.5f, 0f, 0.5f, 0.8f);
                    Gizmos.DrawSphere(finalPoint, 栅格尺寸 * 0.4f);
                }
            }
        }
    }
}