// ============================================================
// 文件名: SimpleForwardModel.cs
// 路径: Assets/Scripts/RL/SimpleForwardModel.cs
// 描述: 好奇心驱动探索的前向动力学模型
// 论文: Path Planning for Unmanned Surface Vehicle Based on 
//       Active Disturbance Rejection PPO (Section 5.3)
// ============================================================

using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using System.Linq;
/// <summary>
/// 轻量级前向动力学模型（好奇心模块核心组件）
/// 预测: s_{t+1} = f_pred(s_t, a_t)
/// 内在奖励: R_curiosity = ||f_pred(s_t, a_t) - s_{t+1}||^2
/// </summary>
[Serializable]
public class SimpleForwardModel
{
    #region 可配置参数
    [Header("网络结构")]
    [Tooltip("输入层维度 = 观测维度 + 动作维度")]
    public int inputDim = 140;              // 134 obs + 6 actions (改为134+6=140)

    [Tooltip("隐层神经元数量")]
    public int hiddenDim = 256;

    [Tooltip("输出层维度 = 观测维度")]
    public int outputDim = 134;             // 改为134

    [Header("训练超参数")]
    [Tooltip("学习率")]
    public float learningRate = 0.001f;

    [Tooltip("批次大小")]
    public int batchSize = 64;

    [Tooltip("训练间隔（步数）")]
    public int trainInterval = 10;

    [Tooltip("最大经验池容量")]
    public int maxBufferSize = 10000;

    [Header("归一化参数")]
    [Tooltip("输入标准差下限（防止除零）")]
    public float epsilon = 1e-6f;
    #endregion

    #region 权重矩阵
    // 三层全连接网络: Input → Hidden1 → Hidden2 → Output
    private float[,] W1;   // [hiddenDim, inputDim]
    private float[] b1;    // [hiddenDim]

    private float[,] W2;   // [hiddenDim, hiddenDim]
    private float[] b2;    // [hiddenDim]

    private float[,] W3;   // [outputDim, hiddenDim]
    private float[] b3;    // [outputDim]

    // 梯度累积（用于批次更新）
    private float[,] gradW1;
    private float[] gradb1;
    private float[,] gradW2;
    private float[] gradb2;
    private float[,] gradW3;
    private float[] gradb3;

    // 动量（Adam 优化器）
    private float[,] mW1, vW1;
    private float[] mb1, vb1;
    private float[,] mW2, vW2;
    private float[] mb2, vb2;
    private float[,] mW3, vW3;
    private float[] mb3, vb3;
    private int adamStep = 0;
    private const float beta1 = 0.9f;
    private const float beta2 = 0.999f;
    private const float adamEpsilon = 1e-8f;
    #endregion

    #region 经验回放缓冲区
    private struct Experience
    {
        public float[] state;
        public float[] action;
        public float[] nextState;
    }
    private List<Experience> buffer = new List<Experience>();
    private int bufferIndex = 0;
    private int stepCounter = 0;

    // 统计归一化
    private float[] inputMean;
    private float[] inputStd;
    private float[] outputMean;
    private float[] outputStd;
    private bool isNormalized = false;
    private int normalizationSamples = 0;
    private const int MIN_NORM_SAMPLES = 100;
    #endregion

    #region 构造函数
    public SimpleForwardModel(int obsDim, int actionDim, int hiddenDim = 256, float lr = 0.001f)
    {
        this.inputDim = obsDim + actionDim;
        this.hiddenDim = hiddenDim;
        this.outputDim = obsDim;
        this.learningRate = lr;

        // 初始化权重（Xavier 初始化）
        W1 = new float[hiddenDim, inputDim];
        b1 = new float[hiddenDim];
        W2 = new float[hiddenDim, hiddenDim];
        b2 = new float[hiddenDim];
        W3 = new float[outputDim, hiddenDim];
        b3 = new float[outputDim];

        XavierInit(W1, hiddenDim, inputDim);
        XavierInit(W2, hiddenDim, hiddenDim);
        XavierInit(W3, outputDim, hiddenDim);
        Array.Clear(b1, 0, b1.Length);
        Array.Clear(b2, 0, b2.Length);
        Array.Clear(b3, 0, b3.Length);

        // 初始化梯度缓存
        gradW1 = new float[hiddenDim, inputDim];
        gradb1 = new float[hiddenDim];
        gradW2 = new float[hiddenDim, hiddenDim];
        gradb2 = new float[hiddenDim];
        gradW3 = new float[outputDim, hiddenDim];
        gradb3 = new float[outputDim];

        // 初始化 Adam 动量
        mW1 = new float[hiddenDim, inputDim];
        vW1 = new float[hiddenDim, inputDim];
        mb1 = new float[hiddenDim];
        vb1 = new float[hiddenDim];
        mW2 = new float[hiddenDim, hiddenDim];
        vW2 = new float[hiddenDim, hiddenDim];
        mb2 = new float[hiddenDim];
        vb2 = new float[hiddenDim];
        mW3 = new float[outputDim, hiddenDim];
        vW3 = new float[outputDim, hiddenDim];
        mb3 = new float[outputDim];
        vb3 = new float[outputDim];

        // 归一化统计
        inputMean = new float[inputDim];
        inputStd = new float[inputDim];
        outputMean = new float[outputDim];
        outputStd = new float[outputDim];
        Array.Fill(inputStd, 1f);
        Array.Fill(outputStd, 1f);

        Debug.Log($"[ForwardModel] 初始化完成: 输入{inputDim} → 隐层{hiddenDim} → 隐层{hiddenDim} → 输出{outputDim}");
    }
    #endregion

    #region 核心方法：前向传播
    /// <summary>
    /// 预测下一状态
    /// </summary>
    public float[] Predict(float[] state, float[] action)
    {
        if (state == null || action == null)
            return null;

        // 拼接输入 [state, action]
        float[] input = new float[inputDim];
        Array.Copy(state, 0, input, 0, state.Length);
        Array.Copy(action, 0, input, state.Length, action.Length);

        // 归一化
        if (isNormalized)
        {
            for (int i = 0; i < inputDim; i++)
                input[i] = (input[i] - inputMean[i]) / (inputStd[i] + epsilon);
        }

        // 前向传播
        float[] h1 = new float[hiddenDim];
        float[] h2 = new float[hiddenDim];
        float[] output = new float[outputDim];

        // Layer 1: Input → Hidden1 (ReLU)
        MatrixMulAddBias(input, W1, b1, h1);
        ReLU(h1);

        // Layer 2: Hidden1 → Hidden2 (ReLU)
        MatrixMulAddBias(h1, W2, b2, h2);
        ReLU(h2);

        // Layer 3: Hidden2 → Output (Linear)
        MatrixMulAddBias(h2, W3, b3, output);

        // 反归一化输出
        if (isNormalized)
        {
            for (int i = 0; i < outputDim; i++)
                output[i] = output[i] * (outputStd[i] + epsilon) + outputMean[i];
        }

        // 裁剪到合理范围（状态值通常 [-2, 2] 范围）
        for (int i = 0; i < output.Length; i++)
            output[i] = Mathf.Clamp(output[i], -5f, 5f);

        return output;
    }

    /// <summary>
    /// 计算好奇心内在奖励
    /// </summary>
    public float ComputeCuriosityReward(float[] state, float[] action, float[] nextState)
    {
        if (state == null || action == null || nextState == null)
            return 0f;

        float[] predicted = Predict(state, action);
        if (predicted == null)
            return 0f;

        // MSE 预测误差
        float mse = 0f;
        for (int i = 0; i < outputDim; i++)
        {
            float diff = predicted[i] - nextState[i];
            mse += diff * diff;
        }
        mse /= outputDim;

        // 保存经验（用于训练）
        AddExperience(state, action, nextState);

        // 定期训练
        stepCounter++;
        if (stepCounter % trainInterval == 0 && buffer.Count >= batchSize)
        {
            TrainBatch();
        }

        return mse; // 论文公式: ||f_pred(s_t, a_t) - s_{t+1}||^2
    }
    #endregion

    #region 经验管理
    private void AddExperience(float[] state, float[] action, float[] nextState)
    {
        // 更新归一化统计
        UpdateNormalization(state, action, nextState);

        // 存入经验池
        if (buffer.Count < maxBufferSize)
        {
            buffer.Add(new Experience
            {
                state = (float[])state.Clone(),
                action = (float[])action.Clone(),
                nextState = (float[])nextState.Clone()
            });
        }
        else
        {
            // 环形缓冲区覆盖
            int idx = bufferIndex % maxBufferSize;
            buffer[idx] = new Experience
            {
                state = (float[])state.Clone(),
                action = (float[])action.Clone(),
                nextState = (float[])nextState.Clone()
            };
            bufferIndex++;
        }
    }

    private void UpdateNormalization(float[] state, float[] action, float[] nextState)
    {
        normalizationSamples++;
        float alpha = 1f / normalizationSamples;

        // 更新输入均值和方差
        for (int i = 0; i < state.Length; i++)
        {
            float oldMean = inputMean[i];
            inputMean[i] += (state[i] - inputMean[i]) * alpha;
            inputStd[i] += (state[i] - oldMean) * (state[i] - inputMean[i]);
        }
        for (int i = 0; i < action.Length; i++)
        {
            int idx = state.Length + i;
            float oldMean = inputMean[idx];
            inputMean[idx] += (action[i] - inputMean[idx]) * alpha;
            inputStd[idx] += (action[i] - oldMean) * (action[i] - inputMean[idx]);
        }

        // 更新输出均值和方差
        for (int i = 0; i < nextState.Length; i++)
        {
            float oldMean = outputMean[i];
            outputMean[i] += (nextState[i] - outputMean[i]) * alpha;
            outputStd[i] += (nextState[i] - oldMean) * (nextState[i] - outputMean[i]);
        }

        if (normalizationSamples >= MIN_NORM_SAMPLES)
        {
            isNormalized = true;
            // 计算标准差
            for (int i = 0; i < inputStd.Length; i++)
                inputStd[i] = Mathf.Sqrt(inputStd[i] / normalizationSamples + epsilon);
            for (int i = 0; i < outputStd.Length; i++)
                outputStd[i] = Mathf.Sqrt(outputStd[i] / normalizationSamples + epsilon);
        }
    }
    #endregion

    #region 训练方法
    /// <summary>
    /// 随机批次训练
    /// </summary>
    private void TrainBatch()
    {
        if (buffer.Count < batchSize) return;

        // 随机采样批次
        List<Experience> batch = new List<Experience>(batchSize);
        for (int i = 0; i < batchSize; i++)
        {
            int idx = UnityEngine.Random.Range(0, buffer.Count);
            batch.Add(buffer[idx]);
        }

        // 清零梯度
        Array.Clear(gradW1, 0, gradW1.Length);
        Array.Clear(gradb1, 0, gradb1.Length);
        Array.Clear(gradW2, 0, gradW2.Length);
        Array.Clear(gradb2, 0, gradb2.Length);
        Array.Clear(gradW3, 0, gradW3.Length);
        Array.Clear(gradb3, 0, gradb3.Length);

        // 累积梯度
        foreach (var exp in batch)
        {
            // 构建输入
            float[] input = new float[inputDim];
            Array.Copy(exp.state, 0, input, 0, exp.state.Length);
            Array.Copy(exp.action, 0, input, exp.state.Length, exp.action.Length);

            // 归一化
            float[] normInput = new float[inputDim];
            for (int i = 0; i < inputDim; i++)
                normInput[i] = isNormalized ? (input[i] - inputMean[i]) / (inputStd[i] + epsilon) : input[i];

            float[] normTarget = new float[outputDim];
            for (int i = 0; i < outputDim; i++)
                normTarget[i] = isNormalized ? (exp.nextState[i] - outputMean[i]) / (outputStd[i] + epsilon) : exp.nextState[i];

            // 前向传播
            float[] h1 = new float[hiddenDim];
            float[] h2 = new float[hiddenDim];
            float[] output = new float[outputDim];

            MatrixMulAddBias(normInput, W1, b1, h1);
            ReLU(h1);

            MatrixMulAddBias(h1, W2, b2, h2);
            ReLU(h2);

            MatrixMulAddBias(h2, W3, b3, output);

            // 计算误差
            float[] delta3 = new float[outputDim];
            for (int i = 0; i < outputDim; i++)
                delta3[i] = output[i] - normTarget[i];

            // 反向传播
            // δ3 → δ2
            float[] delta2 = new float[hiddenDim];
            for (int j = 0; j < hiddenDim; j++)
            {
                float sum = 0f;
                for (int k = 0; k < outputDim; k++)
                    sum += delta3[k] * W3[k, j];
                delta2[j] = sum * (h2[j] > 0 ? 1f : 0f); // ReLU 导数
            }

            // δ2 → δ1
            float[] delta1 = new float[hiddenDim];
            for (int i = 0; i < hiddenDim; i++)
            {
                float sum = 0f;
                for (int j = 0; j < hiddenDim; j++)
                    sum += delta2[j] * W2[j, i];
                delta1[i] = sum * (h1[i] > 0 ? 1f : 0f); // ReLU 导数
            }

            // 累积梯度 (W3, b3)
            for (int i = 0; i < outputDim; i++)
            {
                for (int j = 0; j < hiddenDim; j++)
                    gradW3[i, j] += delta3[i] * h2[j];
                gradb3[i] += delta3[i];
            }

            // 累积梯度 (W2, b2)
            for (int i = 0; i < hiddenDim; i++)
            {
                for (int j = 0; j < hiddenDim; j++)
                    gradW2[i, j] += delta2[i] * h1[j];
                gradb2[i] += delta2[i];
            }

            // 累积梯度 (W1, b1)
            for (int i = 0; i < hiddenDim; i++)
            {
                for (int j = 0; j < inputDim; j++)
                    gradW1[i, j] += delta1[i] * normInput[j];
                gradb1[i] += delta1[i];
            }
        }

        // 平均梯度
        float invBatchSize = 1f / batchSize;
        for (int i = 0; i < hiddenDim; i++)
        {
            for (int j = 0; j < inputDim; j++)
                gradW1[i, j] *= invBatchSize;
            gradb1[i] *= invBatchSize;
        }
        for (int i = 0; i < hiddenDim; i++)
        {
            for (int j = 0; j < hiddenDim; j++)
                gradW2[i, j] *= invBatchSize;
            gradb2[i] *= invBatchSize;
        }
        for (int i = 0; i < outputDim; i++)
        {
            for (int j = 0; j < hiddenDim; j++)
                gradW3[i, j] *= invBatchSize;
            gradb3[i] *= invBatchSize;
        }

        // Adam 更新
        AdamUpdate();
    }

    /// <summary>
    /// Adam 优化器更新
    /// </summary>
    private void AdamUpdate()
    {
        adamStep++;
        float lr = learningRate * Mathf.Sqrt(1f - Mathf.Pow(beta2, adamStep)) /
                   (1f - Mathf.Pow(beta1, adamStep));

        // 更新 W1, b1
        for (int i = 0; i < hiddenDim; i++)
        {
            for (int j = 0; j < inputDim; j++)
            {
                mW1[i, j] = beta1 * mW1[i, j] + (1 - beta1) * gradW1[i, j];
                vW1[i, j] = beta2 * vW1[i, j] + (1 - beta2) * gradW1[i, j] * gradW1[i, j];
                W1[i, j] -= lr * mW1[i, j] / (Mathf.Sqrt(vW1[i, j]) + adamEpsilon);
            }
            mb1[i] = beta1 * mb1[i] + (1 - beta1) * gradb1[i];
            vb1[i] = beta2 * vb1[i] + (1 - beta2) * gradb1[i] * gradb1[i];
            b1[i] -= lr * mb1[i] / (Mathf.Sqrt(vb1[i]) + adamEpsilon);
        }

        // 更新 W2, b2
        for (int i = 0; i < hiddenDim; i++)
        {
            for (int j = 0; j < hiddenDim; j++)
            {
                mW2[i, j] = beta1 * mW2[i, j] + (1 - beta1) * gradW2[i, j];
                vW2[i, j] = beta2 * vW2[i, j] + (1 - beta2) * gradW2[i, j] * gradW2[i, j];
                W2[i, j] -= lr * mW2[i, j] / (Mathf.Sqrt(vW2[i, j]) + adamEpsilon);
            }
            mb2[i] = beta1 * mb2[i] + (1 - beta1) * gradb2[i];
            vb2[i] = beta2 * vb2[i] + (1 - beta2) * gradb2[i] * gradb2[i];
            b2[i] -= lr * mb2[i] / (Mathf.Sqrt(vb2[i]) + adamEpsilon);
        }

        // 更新 W3, b3
        for (int i = 0; i < outputDim; i++)
        {
            for (int j = 0; j < hiddenDim; j++)
            {
                mW3[i, j] = beta1 * mW3[i, j] + (1 - beta1) * gradW3[i, j];
                vW3[i, j] = beta2 * vW3[i, j] + (1 - beta2) * gradW3[i, j] * gradW3[i, j];
                W3[i, j] -= lr * mW3[i, j] / (Mathf.Sqrt(vW3[i, j]) + adamEpsilon);
            }
            mb3[i] = beta1 * mb3[i] + (1 - beta1) * gradb3[i];
            vb3[i] = beta2 * vb3[i] + (1 - beta2) * gradb3[i] * gradb3[i];
            b3[i] -= lr * mb3[i] / (Mathf.Sqrt(vb3[i]) + adamEpsilon);
        }
    }
    #endregion

    #region 辅助数学函数
    private void XavierInit(float[,] matrix, int rows, int cols)
    {
        float scale = Mathf.Sqrt(2f / (rows + cols));
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                matrix[i, j] = (float)(UnityEngine.Random.Range(0f, 1f) * 2 - 1) * scale;
    }

    private void MatrixMulAddBias(float[] input, float[,] weight, float[] bias, float[] output)
    {
        int rows = weight.GetLength(0);
        int cols = weight.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            float sum = bias[i];
            for (int j = 0; j < cols; j++)
                sum += weight[i, j] * input[j];
            output[i] = sum;
        }
    }

    private void ReLU(float[] x)
    {
        for (int i = 0; i < x.Length; i++)
            x[i] = Mathf.Max(0f, x[i]);
    }
    #endregion

    #region 调试与保存
    public void PrintStats()
    {
        Debug.Log($"[ForwardModel] 状态: 经验数={buffer.Count}, 步数={stepCounter}, 归一化={isNormalized}");
        if (isNormalized)
        {
            Debug.Log($"  输入均值: {string.Join(", ", inputMean.Take(5))}...");
            Debug.Log($"  输入标准差: {string.Join(", ", inputStd.Take(5))}...");
        }
    }

    public void ClearBuffer()
    {
        buffer.Clear();
        bufferIndex = 0;
        stepCounter = 0;
        Debug.Log("[ForwardModel] 经验池已清空");
    }
    #endregion
}