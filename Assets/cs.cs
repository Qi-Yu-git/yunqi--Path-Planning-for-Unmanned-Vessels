using UnityEngine;
using OpenCvSharp; // 关键：检查是否报错

public class OpenCvTest : MonoBehaviour
{
    void Start()
    {
        // 尝试创建一个空的Mat对象，验证库是否加载
        using (Mat mat = new Mat())
        {
            Debug.Log("OpenCvSharp加载成功！");
        }
    }
}