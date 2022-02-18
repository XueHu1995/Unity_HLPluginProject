#pragma once
#include "HL2ResearchMode.g.h"
#include "ResearchModeApi.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <wchar.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <future>
#include <cmath>
#include <DirectXMath.h>
#include <vector>
#include<winrt/Windows.Perception.Spatial.h>
#include<winrt/Windows.Perception.Spatial.Preview.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

/**
 * @file HL2ResearchMode.h
 *
 * @brief Header for all functions implemented via HL2ResearchMode class
 *
 * Adapted from online repo by petergu684's HoloLens2-ResearchMode-Unity on GitHub. Additional
 * functionality for processing AB frames to detect the presence of IR-marker equipped arrays/tools
 * 
 * @author Hisham Iqbal
 * @date 2021
 * 
 */

namespace winrt::HL2UnityPlugin::implementation
{
    struct HL2ResearchMode : HL2ResearchModeT<HL2ResearchMode>
    {
        HL2ResearchMode();

        UINT16 GetCenterDepth();
        int GetDepthBufferSize();
        int GetLongDepthBufferSize();
        hstring PrintDepthResolution();
        hstring PrintDepthExtrinsics();
		hstring PrintLFResolution();
		hstring PrintLFExtrinsics();
		hstring PrintRFResolution();
		hstring PrintRFExtrinsics();
        hstring GetDebugTrackedToolsetString();

        // public facing function for setting tool list
        void SetToolListByString(hstring const& tool_list);

        void InitializeDepthSensor();
        void InitializeLongDepthSensor();
        void InitializeSpatialCamerasFront();

        void StartDepthSensorLoop();
        void StartLongDepthSensorLoop();
        void StartSpatialCamerasFrontLoop();

        void StopAllSensorDevice();

        bool DepthMapTextureUpdated();
        bool ShortAbImageTextureUpdated();
        bool PointCloudUpdated();
        bool LongDepthMapTextureUpdated();
		bool LFImageUpdated();
		bool RFImageUpdated();
        bool ThreshAbImageTextureUpdated();
        bool KeypointAbImageTextureUpdated();

        bool ToolDetectedByHoloLens();

        void SetReferenceCoordinateSystem(Windows::Perception::Spatial::SpatialCoordinateSystem refCoord);
        void SetPointCloudRoiInSpace(float centerX, float centerY, float centerZ, float boundX, float boundY, float boundZ);
        void SetPointCloudDepthOffset(uint16_t offset);

        com_array<uint16_t> GetDepthMapBuffer();
        com_array<uint8_t> GetDepthMapTextureBuffer();
        com_array<uint16_t> GetShortAbImageBuffer();
        com_array<uint8_t> GetShortAbImageTextureBuffer();
        com_array<uint16_t> GetLongDepthMapBuffer();
        com_array<uint8_t> GetLongDepthMapTextureBuffer();
		com_array<uint8_t> GetLFCameraBuffer();
		com_array<uint8_t> GetRFCameraBuffer();
        com_array<float> GetPointCloudBuffer();
        com_array<float> GetCenterPoint();
        com_array<float> GetDepthSensorPosition();
        //HI addition
        com_array<float> GetDepthToWorldMatrix();
        com_array<double> GetToolToWorldMatrix();
        com_array<double> GetTrackedToolsPoseMatrices();
        com_array<uint8_t> GetShortThreshAbImageTextureBuffer();
        com_array<uint8_t> GetShortKeypointAbImageTextureBuffer();
        //
        std::mutex mu;

    private:
        // delete these asap
        std::stringstream m_debugStringStream;
        long long m_debugLoopDuration = 0;

        float* m_pointCloud = nullptr;
        int m_pointcloudLength = 0;
        UINT16* m_depthMap = nullptr;
        UINT8* m_depthMapTexture = nullptr;
        UINT16* m_shortAbImage = nullptr;
        UINT8* m_shortAbImageTexture = nullptr;
        UINT16* m_longDepthMap = nullptr;
        UINT8* m_longDepthMapTexture = nullptr;
		UINT8* m_LFImage = nullptr;
		UINT8* m_RFImage = nullptr;
        //HI Additions
        UINT8* m_shortThreshAbImageTexture = nullptr;
        UINT8* m_shortKeypointAbImageTexture = nullptr;
        //
        IResearchModeSensor* m_depthSensor = nullptr;
        IResearchModeCameraSensor* m_pDepthCameraSensor = nullptr;
        IResearchModeSensor* m_longDepthSensor = nullptr;
        IResearchModeCameraSensor* m_pLongDepthCameraSensor = nullptr;
        IResearchModeSensor* m_LFSensor = nullptr;
        IResearchModeCameraSensor* m_LFCameraSensor = nullptr;
        IResearchModeSensor* m_RFSensor = nullptr;
        IResearchModeCameraSensor* m_RFCameraSensor = nullptr;
        ResearchModeSensorResolution m_depthResolution;
        ResearchModeSensorResolution m_longDepthResolution;
        ResearchModeSensorResolution m_LFResolution;
        ResearchModeSensorResolution m_RFResolution;
        IResearchModeSensorDevice* m_pSensorDevice = nullptr;
        std::vector<ResearchModeSensorDescriptor> m_sensorDescriptors;
        IResearchModeSensorDeviceConsent* m_pSensorDeviceConsent = nullptr;
        Windows::Perception::Spatial::SpatialLocator m_locator = 0;
        Windows::Perception::Spatial::SpatialCoordinateSystem m_refFrame = nullptr;
        std::atomic_int m_depthBufferSize = 0;
        std::atomic_int m_longDepthBufferSize = 0;
        std::atomic_int m_LFbufferSize = 0;
        std::atomic_int m_RFbufferSize = 0;
        std::atomic_uint16_t m_centerDepth = 0;
        float m_centerPoint[3]{ 0,0,0 };
        float m_depthSensorPosition[3]{ 0,0,0 };
        //HI Addition
        float m_depthToWorldMat[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
        double m_toolToWorldMat[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
        //
        std::atomic_bool m_depthSensorLoopStarted = false;
        std::atomic_bool m_longDepthSensorLoopStarted = false;
        std::atomic_bool m_spatialCamerasFrontLoopStarted = false;
        std::atomic_bool m_depthMapTextureUpdated = false;
        std::atomic_bool m_shortAbImageTextureUpdated = false;
        std::atomic_bool m_longDepthMapTextureUpdated = false;
        std::atomic_bool m_pointCloudUpdated = false;
        std::atomic_bool m_useRoiFilter = false;
		std::atomic_bool m_LFImageUpdated = false;
		std::atomic_bool m_RFImageUpdated = false;
        // HI Additions:
        std::atomic_bool m_threshAbImageTextureUpdated = false;
        std::atomic_bool m_keypointAbImageTextureUpdated = false;
        std::atomic_bool m_toolDetectedByHololens = false; // flag for reading m_ToolList
        //
        float m_roiBound[3]{ 0,0,0 };
        float m_roiCenter[3]{ 0,0,0 };
        static void SegmentationDepthSensorLoop(HL2ResearchMode* pHL2ResearchMode);
        static void LongDepthSensorLoop(HL2ResearchMode* pHL2ResearchMode);
        static void SpatialCamerasFrontLoop(HL2ResearchMode* pHL2ResearchMode);
        static void CamAccessOnComplete(ResearchModeSensorConsent consent);
        std::string MatrixToString(DirectX::XMFLOAT4X4 mat);
        DirectX::XMFLOAT4X4 m_depthCameraPose;
        DirectX::XMMATRIX m_depthCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_longDepthCameraPose;
        DirectX::XMMATRIX m_longDepthCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_LFCameraPose;
        DirectX::XMMATRIX m_LFCameraPoseInvMatrix;
        DirectX::XMFLOAT4X4 m_RFCameraPose;
        DirectX::XMMATRIX m_RFCameraPoseInvMatrix;
        std::thread* m_pDepthUpdateThread;
        std::thread* m_pLongDepthUpdateThread;
        std::thread* m_pSpatialCamerasFrontUpdateThread;
        static long long checkAndConvertUnsigned(UINT64 val);
        struct DepthCamRoi {
            float kRowLower = 0.2;
            float kRowUpper = 0.5;
            float kColLower = 0.3;
            float kColUpper = 0.7;
            UINT16 depthNearClip = 200; // Unit: mm
            UINT16 depthFarClip = 800;
        } depthCamRoi;
        UINT16 m_depthOffset = 0;

        // Extra Hisham functions

        static void Eigen_Vec2DX_Vec(std::vector<Eigen::Vector3d>& Input_CV_Vec, std::vector<DirectX::XMVECTOR>& Output_DX_Vec);

        static bool PruneDuplicates(std::vector<DirectX::XMVECTOR>& ioPointList);

        static float GetVectorLength(DirectX::XMVECTOR xmVec);

        static bool GenerateCandidatesList(std::vector<std::vector<int>>& inOptionList, int unregPointsCount);

        static bool PruneByDistance(float inDistance, std::vector<std::vector<int>>& inOptionList, DirectX::XMVECTOR* unregisteredPoints);

        static bool GetPointCorrespondence(std::vector<Eigen::Vector3d> ReferencePoints, std::vector<Eigen::Vector3d> CollectedPoints, 
            std::vector<std::vector<int>>& CorrespondenceList);

        // New Hisham Functions
        struct InfraredKeypoint
        {
            Eigen::Vector3d WorldLocation; // position of marker w.r.t. holographic world frame
            Eigen::Vector3d DepthLocation; // position of marker w.r.t depth sensor
            cv::KeyPoint ImageKeypoint; // 2.D pixel location of marker
        };

        struct TrackedTool
        {
            int ID;
            std::vector<Eigen::Vector3d> GeometryPoints; // Known coordinates of tool-marker positions (from CAD file, config files etc.)
            std::vector<Eigen::Vector3d> ObservedPoints_World; // Observed tool marker points in world frame (should be the same order as GeometryPoints)
            std::vector<Eigen::Vector3d> ObservedPoints_Depth; // Observed tool marker points in depth-sensor frame (should be the same order as GeometryPoints)
            bool VisibleToHoloLens;
            Eigen::Matrix4d PoseMatrix_HoloWorld;
            Eigen::Matrix4d PoseMatrix_DepthCamera;
            std::vector<cv::KeyPoint> ObservedImgKeypoints; // Image coordinates for marker-centres (should be the same order as GeometryPoints)
        };

        std::vector<TrackedTool> m_ToolList; // The tools we want to track + update
        std::vector<InfraredKeypoint> m_InfraredBlobs; // Observed blobs + coordinates

        // formatted double array with info of each tracked tool 
        // transform matrix stored as row major
        // [toolID,toolVisible(0/1),mat00,mat01,mat02...,mat33]
        double* m_OutputToolPoseList = nullptr;
        int m_OutputToolPoseListLen = 0;

        // function used to load in tool triplet information with a encoded string
        void PopulateToolList(std::string encoded_String, std::vector<TrackedTool>& toolList);
        
        static void TryLocatingTools(std::vector<InfraredKeypoint> infraredBlobs, std::vector<TrackedTool>& tools);
        static void GetFormattedDoubleMat(std::vector<TrackedTool> tools, double* outputDoubleArr);
        static bool PruneByDistance(float inDistance, std::vector<std::vector<int>>& inOptionList, Eigen::Vector3d* unregisteredPoints);
        static bool PruneDuplicates(std::vector<Eigen::Vector3d>& ioPointList);

    };
}
namespace winrt::HL2UnityPlugin::factory_implementation
{
    struct HL2ResearchMode : HL2ResearchModeT<HL2ResearchMode, implementation::HL2ResearchMode>
    {
    };
}
