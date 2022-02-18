#include "pch.h"
#include "HL2ResearchMode.h"
#include "HL2ResearchMode.g.cpp"

#include <algorithm>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
//#include <opencv2/opencv.hpp>

extern "C"
HMODULE LoadLibraryA(
    LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;

using namespace DirectX;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

typedef std::chrono::duration<int64_t, std::ratio<1, 10'000'000>> HundredsOfNanoseconds;

/**
 * @file HL2ResearchMode.cpp
 *
 * @brief Implementations for all functions implemented via HL2ResearchMode class
 *
 * @author Hisham Iqbal
 * @date 2021
 */

namespace winrt::HL2UnityPlugin::implementation
{
    template <typename T>
    cv::Mat createMat(const T* data, int rows, int cols, int chs = 1) {
        // Create Mat from buffer 
        cv::Mat mat(rows, cols, CV_MAKETYPE(cv::DataType<T>::type, chs));
        memcpy(mat.data, data, static_cast<unsigned long long>(rows) * cols * chs * sizeof(T));
        return mat;
    }

    typedef std::vector<Eigen::Vector3d> PointsType;

    Eigen::Matrix4d ComputeRigidTransform(const PointsType& src, const PointsType& dst)
    {
        // computes the rigid transformation that transforms src TO dst:
        // i.e. T * src = dst, src=model

        Eigen::Matrix4d returnMat = Eigen::Matrix4d::Identity();

        if (src.size() != dst.size()) { return returnMat; }
        int pairSize = src.size();
        Eigen::Vector3d center_src(0, 0, 0), center_dst(0, 0, 0);
        for (int i = 0; i < pairSize; ++i)
        {
            center_src += src[i];
            center_dst += dst[i];
        }
        center_src /= (double)pairSize;
        center_dst /= (double)pairSize;

        Eigen::MatrixXd S(pairSize, 3), D(pairSize, 3);
        for (int i = 0; i < pairSize; ++i)
        {
            for (int j = 0; j < 3; ++j)
                S(i, j) = src[i][j] - center_src[j];
            for (int j = 0; j < 3; ++j)
                D(i, j) = dst[i][j] - center_dst[j];
        }
        Eigen::MatrixXd Dt = D.transpose();
        Eigen::Matrix3d H = Dt * S;
        Eigen::Matrix3d W, U, V;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd;
        Eigen::MatrixXd H_(3, 3);
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) H_(i, j) = H(i, j);
        svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (!svd.computeU() || !svd.computeV()) {
            //std::cerr << "decomposition error" << std::endl;
            //return std::make_pair(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
            return returnMat;
        }
        Eigen::Matrix3d Vt = svd.matrixV().transpose();
        Eigen::Matrix3d R = svd.matrixU() * Vt;

        if (R.determinant() < 0.) {
            //std::cout << "Reflection detected..." << endl;
            Vt(2, 0) *= -1.;
            Vt(2, 1) *= -1.;
            Vt(2, 2) *= -1.;
            R = svd.matrixU() * Vt;
        }

        Eigen::Vector3d t = center_dst - R * center_src;

        returnMat.block(0, 0, 3, 3) = R;
        returnMat.block(0, 3, 3, 1) = t;

        return returnMat;
    }

    HL2ResearchMode::HL2ResearchMode() 
    {
        // Load Research Mode library
        camConsentGiven = CreateEvent(nullptr, true, false, nullptr);
        HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
        HRESULT hr = S_OK;

        if (hrResearchMode)
        {
            typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
            PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
            if (pfnCreate)
            {
                winrt::check_hresult(pfnCreate(&m_pSensorDevice));
            }
            else
            {
                winrt::check_hresult(E_INVALIDARG);
            }
        }

        // get spatial locator of rigNode
        GUID guid;
        IResearchModeSensorDevicePerception* pSensorDevicePerception;
        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
        winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&guid));
        pSensorDevicePerception->Release();
        m_locator = SpatialGraphInteropPreview::CreateLocatorForNode(guid);

        size_t sensorCount = 0;

        winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
        winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(HL2ResearchMode::CamAccessOnComplete));

        m_pSensorDevice->DisableEyeSelection();

        winrt::check_hresult(m_pSensorDevice->GetSensorCount(&sensorCount));
        m_sensorDescriptors.resize(sensorCount);
        winrt::check_hresult(m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(), m_sensorDescriptors.size(), &sensorCount));

        //std::string toolList = "0,0.000000,0.000000,0.008900,-0.007670,0.051440,0.008900,0.097330,0.051260,0.008900,0.145010,0.000000,0.008900;1,-0.045010,0.036830,0.023070,-0.038890,0.148820,0.023070,0.040560,0.168630,0.023070,0.043990,0.048690,0.023070";
        //PopulateToolList(toolList, m_ToolList);
    }

    void HL2ResearchMode::InitializeDepthSensor() 
    {
       
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == DEPTH_AHAT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_depthSensor));
                winrt::check_hresult(m_depthSensor->QueryInterface(IID_PPV_ARGS(&m_pDepthCameraSensor)));
                winrt::check_hresult(m_pDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_depthCameraPose));
                m_depthCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_depthCameraPose));
                break;
            }
        }
    }

    void HL2ResearchMode::InitializeLongDepthSensor()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == DEPTH_LONG_THROW)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_longDepthSensor));
                winrt::check_hresult(m_longDepthSensor->QueryInterface(IID_PPV_ARGS(&m_pLongDepthCameraSensor)));
                winrt::check_hresult(m_pLongDepthCameraSensor->GetCameraExtrinsicsMatrix(&m_longDepthCameraPose));
                m_longDepthCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_longDepthCameraPose));
                break;
            }
        }
    }

    void HL2ResearchMode::InitializeSpatialCamerasFront()
    {
        for (auto sensorDescriptor : m_sensorDescriptors)
        {
            if (sensorDescriptor.sensorType == LEFT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_LFSensor));
                winrt::check_hresult(m_LFSensor->QueryInterface(IID_PPV_ARGS(&m_LFCameraSensor)));
                winrt::check_hresult(m_LFCameraSensor->GetCameraExtrinsicsMatrix(&m_LFCameraPose));
                m_LFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_LFCameraPose));
            }
            if (sensorDescriptor.sensorType == RIGHT_FRONT)
            {
                winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_RFSensor));
                winrt::check_hresult(m_RFSensor->QueryInterface(IID_PPV_ARGS(&m_RFCameraSensor)));
                winrt::check_hresult(m_RFCameraSensor->GetCameraExtrinsicsMatrix(&m_RFCameraPose));
                m_RFCameraPoseInvMatrix = XMMatrixInverse(nullptr, XMLoadFloat4x4(&m_RFCameraPose));
            }
        }
    }

    void HL2ResearchMode::StartDepthSensorLoop() 
    {
        if (m_refFrame == nullptr) 
        {
            // world frame spatial locator, initialised when no previous frame exists
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();            
        }

        m_pDepthUpdateThread = new std::thread(HL2ResearchMode::SegmentationDepthSensorLoop, this);
    }

    void HL2ResearchMode::SegmentationDepthSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_depthSensorLoopStarted)
        {
            pHL2ResearchMode->m_depthSensorLoopStarted = true;
        }
        else {
            return;
        }

        pHL2ResearchMode->m_depthSensor->OpenStream();

        try
        {
            // Blob detector setup
            cv::SimpleBlobDetector::Params params;
            params.filterByArea = true;
            params.minArea = 1;
            params.maxArea = 1000;
            params.filterByColor = true;
            params.blobColor = 255;
            params.filterByConvexity = false;
            params.filterByCircularity = true;
            params.minCircularity = 0.8f;
            params.filterByInertia = true;
            params.minInertiaRatio = 0.5f;

            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

            while (pHL2ResearchMode->m_depthSensorLoopStarted)
            {
                auto start = std::chrono::high_resolution_clock::now();
                pHL2ResearchMode->m_debugStringStream.str("Debug String\n");

                IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
                ResearchModeSensorResolution resolution;
                pHL2ResearchMode->m_depthSensor->GetNextBuffer(&pDepthSensorFrame);

                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2ResearchMode->m_depthResolution = resolution;

                // refresh depth frame for active brightness + depth frames
                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);
                pHL2ResearchMode->m_depthBufferSize = outBufferCount;
                size_t outAbBufferCount = 0;
                const UINT16* pAbImage = nullptr;
                pDepthFrame->GetAbDepthBuffer(&pAbImage, &outAbBufferCount);

                // Raw textures to store ab + depth frames as images
                auto pDepthTexture = std::make_unique<uint8_t[]>(outBufferCount);
                auto pAbTexture = std::make_unique<uint8_t[]>(outAbBufferCount);

                // HI Additions - c_thresh and c_kp mats are for contiguous mats so they can be copied into memory fast
                cv::Mat abImageMat, abImage8bit, threshAbImage8bit, c_threshABimg, c_kpABimg;
                abImage8bit = cv::Mat(512, 512, CV_8UC1);
                threshAbImage8bit = cv::Mat(512, 512, CV_8UC1);
                c_kpABimg = cv::Mat(512, 512, CV_8UC1);
                //

                std::vector<float> pointCloud; // will store 3D world locations of detected blob-centres

                // get tracking transform
                ResearchModeSensorTimestamp timestamp;
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                
                // calculate the pose of headset rig coordinate system w.r.t world frame
                auto transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame); 
                if (transToWorld == nullptr)
                {
                    continue;
                }

                auto rot = transToWorld.Orientation();
                auto quatInDx = XMFLOAT4(rot.x, rot.y, rot.z, rot.w);
                auto rotMat = XMMatrixRotationQuaternion(XMLoadFloat4(&quatInDx));
                auto pos = transToWorld.Position();
                auto posMat = XMMatrixTranslation(pos.x, pos.y, pos.z);

                // if headset-rig pose is known, multiply by the extrinsic transform between the depthCamera and LF sensor
                // calculates the depthToWorld matrix ???
                auto depthToWorld = pHL2ResearchMode->m_depthCameraPoseInvMatrix * rotMat * posMat;
                
                // initialising a XMFLOAT4X4 to allow access to individual elements + update m_depthToWorld
                XMFLOAT4X4 tmpdepth2WorldMat;
                XMStoreFloat4x4(&tmpdepth2WorldMat, depthToWorld);
                float tmpMatValues[16] = { tmpdepth2WorldMat._11, tmpdepth2WorldMat._12, tmpdepth2WorldMat._13, tmpdepth2WorldMat._14,
                                           tmpdepth2WorldMat._21, tmpdepth2WorldMat._22, tmpdepth2WorldMat._23, tmpdepth2WorldMat._24,
                                           tmpdepth2WorldMat._31, tmpdepth2WorldMat._32, tmpdepth2WorldMat._33, tmpdepth2WorldMat._34,
                                           tmpdepth2WorldMat._41, tmpdepth2WorldMat._42, tmpdepth2WorldMat._43, tmpdepth2WorldMat._44 };

                Eigen::Matrix4d eig_depthToWorld, eig_world2depth;
                eig_depthToWorld << 
                    (double)tmpdepth2WorldMat._11, (double)tmpdepth2WorldMat._12, (double)tmpdepth2WorldMat._13, (double)tmpdepth2WorldMat._14,
                    (double)tmpdepth2WorldMat._21, (double)tmpdepth2WorldMat._22, (double)tmpdepth2WorldMat._23, (double)tmpdepth2WorldMat._24,
                    (double)tmpdepth2WorldMat._31, (double)tmpdepth2WorldMat._32, (double)tmpdepth2WorldMat._33, (double)tmpdepth2WorldMat._34,
                    (double)tmpdepth2WorldMat._41, (double)tmpdepth2WorldMat._42, (double)tmpdepth2WorldMat._43, (double)tmpdepth2WorldMat._44;

                eig_world2depth = eig_depthToWorld.inverse().eval();

                pHL2ResearchMode->mu.lock();

                // updates the outward facing depthtoworld matrix variable
                std::copy(std::begin(tmpMatValues), std::end(tmpMatValues), std::begin(pHL2ResearchMode->m_depthToWorldMat));

                auto roiCenterFloat = XMFLOAT3(pHL2ResearchMode->m_roiCenter[0], pHL2ResearchMode->m_roiCenter[1], pHL2ResearchMode->m_roiCenter[2]);
                auto roiBoundFloat = XMFLOAT3(pHL2ResearchMode->m_roiBound[0], pHL2ResearchMode->m_roiBound[1], pHL2ResearchMode->m_roiBound[2]);

                pHL2ResearchMode->mu.unlock();

                // store raw AB img texture as a CV::mat
                abImageMat = createMat<UINT16>(pAbImage, 512, 512);
                //process image pixels up and down = divide by 1000 * 65535 (2^16 - 1): 
                abImageMat = abImageMat * 65.535f;

                // convert 16 to 8 bit grayscale img
                abImage8bit = cv::Mat(512, 512, CV_8UC1);
                abImageMat.convertTo(abImage8bit, CV_8UC1, 0.00390625f);

                // create a thresholded img (only keep pixels with values between 180-255) and blob detection
                cv::threshold(abImage8bit, threshAbImage8bit, 180, 255, cv::THRESH_BINARY);
                std::vector<cv::KeyPoint> keypoints(20); // initialising with space for 20 keypoints
                detector->detect(threshAbImage8bit, keypoints); // blob detection

                pHL2ResearchMode->m_InfraredBlobs.clear();
                pHL2ResearchMode->m_InfraredBlobs.reserve(keypoints.size() + 10);

                // create contiguous copy of thresholded image and label a contiguous copy of thresholded img
                c_threshABimg = threshAbImage8bit.clone();
                c_kpABimg = threshAbImage8bit.clone();

                // HI ADDITION ???
                // iterate through keypoints and draw circular + cross labels, also add keypoint/marker centres to output pt cloud
                for (size_t i = 0; i != keypoints.size(); i++)
                {
                    float xy[2] = { 0, 0 };
                    float uv[2] = { keypoints[i].pt.x, keypoints[i].pt.y };

                    InfraredKeypoint blob;
                    // annotate img
                    cv::circle(c_kpABimg, keypoints[i].pt, keypoints[i].size + 8, cv::Scalar(160, 160, 160), 3);

                    // unmap pixel location to unit plane
                    pHL2ResearchMode->m_pDepthCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

                    auto idx = (uint)((512 * (size_t)uv[1]) + (size_t)uv[0]); // check if rounding here causes issues, keypoints are floats
                    UINT16 depth = pDepth[idx];
                    depth = (depth > 4090) ? 0 : depth - pHL2ResearchMode->m_depthOffset;

                    auto pointOnUnitPlane = XMFLOAT3(xy[0], xy[1], 1);
                    auto tempPoint = (float)depth / 1000 * XMVector3Normalize(XMLoadFloat3(&pointOnUnitPlane));
                    //tempPoint = XMVectorSetZ(tempPoint, XMVectorGetZ(tempPoint) - 0.010); // fudge factor
                    
                    blob.DepthLocation = Eigen::Vector3d((double)XMVectorGetX(tempPoint),
                        (double)XMVectorGetY(tempPoint),
                        (double)XMVectorGetZ(tempPoint));

                    // apply transformation !!!
                    auto pointInWorld = XMVector3Transform(tempPoint, depthToWorld);

                    if (depth > pHL2ResearchMode->depthCamRoi.depthNearClip && depth < pHL2ResearchMode->depthCamRoi.depthFarClip)
                    {
                        // HI ADDITION - check this casting doesn't cause memory issues

                        // this is right-handed
                        blob.WorldLocation = Eigen::Vector3d 
                        ((double)XMVectorGetX(pointInWorld),
                        (double)XMVectorGetY(pointInWorld),
                        (double)XMVectorGetZ(pointInWorld)); 

                        blob.ImageKeypoint = keypoints[i];

                        pHL2ResearchMode->m_InfraredBlobs.push_back(blob);

                        //// This point cloud is displayed directly in Unity, hence the -z axis (right to left-handed conversion)
                        // this is left-handed
                        pointCloud.push_back(XMVectorGetX(pointInWorld));
                        pointCloud.push_back(XMVectorGetY(pointInWorld));
                        pointCloud.push_back(-XMVectorGetZ(pointInWorld));
                    }                  
                }

                for (UINT i = 0; i < resolution.Height; i++)
                {
                    for (UINT j = 0; j < resolution.Width; j++)
                    {
                        auto idx = resolution.Width * i + j;
                        UINT16 depth = pDepth[idx];
                        depth = (depth > 4090) ? 0 : depth - pHL2ResearchMode->m_depthOffset;

                        // save depth map as grayscale texture pixel into temp buffer
                        if (depth == 0) { pDepthTexture.get()[idx] = 0; }
                        else { pDepthTexture.get()[idx] = (uint8_t)((float)depth / 1000 * 255); }

                        // save AbImage as grayscale texture pixel into temp buffer
                        UINT16 abValue = pAbImage[idx];
                        uint8_t processedAbValue = 0;
                        
                        // flooring any bright pixels to have a maximum value of 255
                        if (abValue > 1000) { processedAbValue = 0xFF; }

                        // rescaling pixel values from 0-1000 to 0-255 (purely for image texture purposes)
                        else { processedAbValue = (uint8_t)((float)abValue / 1000 * 255); }

                        pAbTexture.get()[idx] = processedAbValue;

                        // save the depth of center pixel. Note: not used for anything, preserved from old codebase
                        if (i == (UINT)(0.35 * resolution.Height) && j == (UINT)(0.5 * resolution.Width)
                            && pointCloud.size() >= 3)
                        {
                            pHL2ResearchMode->m_centerDepth = depth;
                            if (depth > pHL2ResearchMode->depthCamRoi.depthNearClip && depth < pHL2ResearchMode->depthCamRoi.depthFarClip)
                            {
                                std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);
                                pHL2ResearchMode->m_centerPoint[0] = *(pointCloud.end() - 3);
                                pHL2ResearchMode->m_centerPoint[1] = *(pointCloud.end() - 2);
                                pHL2ResearchMode->m_centerPoint[2] = *(pointCloud.end() - 1);
                            }
                        }
                    }
                }

                // HI Addition -- Big addition
                // Eigen SVD done internally:

                // !!!! Crucial function which:
                // 1) tries to detect if the tools in m_ToolList are visible in m_InfraredBlobs
                // 2) if the tool is visible, then it calculates the tool pose w.r.t the holographic world frame
                TryLocatingTools(pHL2ResearchMode->m_InfraredBlobs, pHL2ResearchMode->m_ToolList);

                // Annotate found tools
                bool noToolFound = true;

                for(TrackedTool &var : pHL2ResearchMode->m_ToolList)
                {
                    if (var.VisibleToHoloLens)
                    {
                        var.PoseMatrix_DepthCamera = eig_world2depth * var.PoseMatrix_HoloWorld.transpose(); 
                        // transpose is used here because PoseMatrix is automatically transposed for row-major storage purposes

                        if (noToolFound) { noToolFound = false; }
                        int i = 0; 
                        for (const auto& kp : var.ObservedImgKeypoints)
                        {
                            cv::drawMarker(c_kpABimg, kp.pt, cv::Scalar(160, 160, 160), cv::MARKER_CROSS, kp.size + 20, 5);
                            cv::putText(c_kpABimg, std::to_string(i), kp.pt + cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(160, 160, 160), 3);
                            i++;

                        }
                    }
                }

                if (noToolFound)
                {
                    cv::putText(c_kpABimg, cv::String("Tool not found"), cv::Point2f(30, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8f, cv::Scalar(160, 160, 160), 2);

                }
                                
                // copy matrix for tool_to_holoWorld to class variable
                pHL2ResearchMode->mu.lock();
                GetFormattedDoubleMat(pHL2ResearchMode->m_ToolList, pHL2ResearchMode->m_OutputToolPoseList);
                pHL2ResearchMode->mu.unlock();

                pHL2ResearchMode->m_toolDetectedByHololens = true; // flag to say that m_ToolList is ready to be read now

                // ------------------------------------------------


                //DEBUG PRINTS
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                pHL2ResearchMode->m_debugLoopDuration = duration.count();

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    // save point cloud
                    if (!pHL2ResearchMode->m_pointCloud)
                    {
                        OutputDebugString(L"Create Space for point cloud...\n");
                        pHL2ResearchMode->m_pointCloud = new float[outBufferCount * 3];
                    }

                    memcpy(pHL2ResearchMode->m_pointCloud, pointCloud.data(), pointCloud.size() * sizeof(float));
                    pHL2ResearchMode->m_pointcloudLength = pointCloud.size();

                    // save raw depth map
                    if (!pHL2ResearchMode->m_depthMap)
                    {
                        OutputDebugString(L"Create Space for depth map...\n");
                        pHL2ResearchMode->m_depthMap = new UINT16[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_depthMap, pDepth, outBufferCount * sizeof(UINT16));

                    // save pre-processed depth map texture (for visualization)
                    if (!pHL2ResearchMode->m_depthMapTexture)
                    {
                        OutputDebugString(L"Create Space for depth map texture...\n");
                        pHL2ResearchMode->m_depthMapTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_depthMapTexture, pDepthTexture.get(), outBufferCount * sizeof(UINT8));

                    // save raw AbImage
                    if (!pHL2ResearchMode->m_shortAbImage)
                    {
                        OutputDebugString(L"Create Space for short AbImage...\n");
                        pHL2ResearchMode->m_shortAbImage = new UINT16[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_shortAbImage, pAbImage, outBufferCount * sizeof(UINT16));

                    // save pre-processed AbImage texture (for visualization)
                    if (!pHL2ResearchMode->m_shortAbImageTexture)
                    {
                        OutputDebugString(L"Create Space for short AbImage texture...\n");
                        pHL2ResearchMode->m_shortAbImageTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_shortAbImageTexture, pAbTexture.get(), outBufferCount * sizeof(UINT8));

                    // save processed imgs HI Addition
                    if (!pHL2ResearchMode->m_shortKeypointAbImageTexture)
                    {
                        pHL2ResearchMode->m_shortKeypointAbImageTexture = new UINT8[outBufferCount];
                    }
                    std::copy(c_kpABimg.ptr(), c_kpABimg.ptr() + outBufferCount, pHL2ResearchMode->m_shortKeypointAbImageTexture);

                    if (!pHL2ResearchMode->m_shortThreshAbImageTexture)
                    {
                        pHL2ResearchMode->m_shortThreshAbImageTexture = new UINT8[outBufferCount];
                    }
                    std::copy(c_threshABimg.ptr(), c_threshABimg.ptr() + outBufferCount, pHL2ResearchMode->m_shortThreshAbImageTexture);
                    //
                }

                pHL2ResearchMode->m_shortAbImageTextureUpdated = true;
                pHL2ResearchMode->m_depthMapTextureUpdated = true;
                pHL2ResearchMode->m_pointCloudUpdated = true;

                //HI Addition
                pHL2ResearchMode->m_keypointAbImageTextureUpdated = true;
                pHL2ResearchMode->m_threshAbImageTextureUpdated = true;
                c_kpABimg.release(); c_threshABimg.release(); threshAbImage8bit.release();
                abImage8bit.release(); abImageMat.release();
                //

                pDepthTexture.reset();

                // release space
                if (pDepthFrame) {
                    pDepthFrame->Release();
                }
                if (pDepthSensorFrame)
                {
                    pDepthSensorFrame->Release();
                }


            }
        }
        catch (...) {}
        pHL2ResearchMode->m_depthSensor->CloseStream();
        pHL2ResearchMode->m_depthSensor->Release();
        pHL2ResearchMode->m_depthSensor = nullptr;
    }

    void HL2ResearchMode::StartLongDepthSensorLoop()
    {
        if (m_refFrame == nullptr)
        {
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }

        m_pLongDepthUpdateThread = new std::thread(HL2ResearchMode::LongDepthSensorLoop, this);
    }

    void HL2ResearchMode::LongDepthSensorLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_longDepthSensorLoopStarted)
        {
            pHL2ResearchMode->m_longDepthSensorLoopStarted = true;
        }
        else {
            return;
        }

        pHL2ResearchMode->m_longDepthSensor->OpenStream();

        try
        {
            while (pHL2ResearchMode->m_longDepthSensorLoopStarted)
            {
                IResearchModeSensorFrame* pDepthSensorFrame = nullptr;
                ResearchModeSensorResolution resolution;
                pHL2ResearchMode->m_longDepthSensor->GetNextBuffer(&pDepthSensorFrame);

                // process sensor frame
                pDepthSensorFrame->GetResolution(&resolution);
                pHL2ResearchMode->m_longDepthResolution = resolution;

                IResearchModeSensorDepthFrame* pDepthFrame = nullptr;
                winrt::check_hresult(pDepthSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame)));

                size_t outBufferCount = 0;
                const UINT16* pDepth = nullptr;

                const BYTE* pSigma = nullptr;
                pDepthFrame->GetSigmaBuffer(&pSigma, &outBufferCount);
                pDepthFrame->GetBuffer(&pDepth, &outBufferCount);
                pHL2ResearchMode->m_longDepthBufferSize = outBufferCount;

                auto pDepthTexture = std::make_unique<uint8_t[]>(outBufferCount);

                // get tracking transform
                ResearchModeSensorTimestamp timestamp;
                pDepthSensorFrame->GetTimeStamp(&timestamp);

                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                auto transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                if (transToWorld == nullptr)
                {
                    continue;
                }

                for (UINT i = 0; i < resolution.Height; i++)
                {
                    for (UINT j = 0; j < resolution.Width; j++)
                    {
                        auto idx = resolution.Width * i + j;
                        UINT16 depth = pDepth[idx];
                        depth = (pSigma[idx] & 0x80) ? 0 : depth - pHL2ResearchMode->m_depthOffset;  //offset???

                        // save as grayscale texture pixel into temp buffer
                        if (depth == 0) { pDepthTexture.get()[idx] = 0; }
                        else { pDepthTexture.get()[idx] = (uint8_t)((float)depth / 4000 * 255); }
                    }
                }

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

                    // save raw depth map
                    if (!pHL2ResearchMode->m_longDepthMap)
                    {
                        OutputDebugString(L"Create Space for depth map...\n");
                        pHL2ResearchMode->m_longDepthMap = new UINT16[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_longDepthMap, pDepth, outBufferCount * sizeof(UINT16));

                    // save pre-processed depth map texture (for visualization)
                    if (!pHL2ResearchMode->m_longDepthMapTexture)
                    {
                        OutputDebugString(L"Create Space for depth map texture...\n");
                        pHL2ResearchMode->m_longDepthMapTexture = new UINT8[outBufferCount];
                    }
                    memcpy(pHL2ResearchMode->m_longDepthMapTexture, pDepthTexture.get(), outBufferCount * sizeof(UINT8));
                }

                pHL2ResearchMode->m_longDepthMapTextureUpdated = true;

                pDepthTexture.reset();

                // release space
                if (pDepthFrame) {
                    pDepthFrame->Release();
                }
                if (pDepthSensorFrame)
                {
                    pDepthSensorFrame->Release();
                }

            }
        }
        catch (...) {}
        pHL2ResearchMode->m_longDepthSensor->CloseStream();
        pHL2ResearchMode->m_longDepthSensor->Release();
        pHL2ResearchMode->m_longDepthSensor = nullptr;

    }

    void HL2ResearchMode::StartSpatialCamerasFrontLoop()
    {
        if (m_refFrame == nullptr)
        {
            m_refFrame = m_locator.GetDefault().CreateStationaryFrameOfReferenceAtCurrentLocation().CoordinateSystem();
        }

        m_pSpatialCamerasFrontUpdateThread = new std::thread(HL2ResearchMode::SpatialCamerasFrontLoop, this);
    }

    void HL2ResearchMode::SpatialCamerasFrontLoop(HL2ResearchMode* pHL2ResearchMode)
    {
        // prevent starting loop for multiple times
        if (!pHL2ResearchMode->m_spatialCamerasFrontLoopStarted)
        {
            pHL2ResearchMode->m_spatialCamerasFrontLoopStarted = true;
        }
        else {
            return;
        }

        pHL2ResearchMode->m_LFSensor->OpenStream();
        pHL2ResearchMode->m_RFSensor->OpenStream();

        try
        {
            while (pHL2ResearchMode->m_spatialCamerasFrontLoopStarted)
            {
                IResearchModeSensorFrame* pLFCameraFrame = nullptr;
                IResearchModeSensorFrame* pRFCameraFrame = nullptr;
                ResearchModeSensorResolution LFResolution;
                ResearchModeSensorResolution RFResolution;
                pHL2ResearchMode->m_LFSensor->GetNextBuffer(&pLFCameraFrame);
				pHL2ResearchMode->m_RFSensor->GetNextBuffer(&pRFCameraFrame);

                // process sensor frame
                pLFCameraFrame->GetResolution(&LFResolution);
                pHL2ResearchMode->m_LFResolution = LFResolution;
                pRFCameraFrame->GetResolution(&RFResolution);
                pHL2ResearchMode->m_RFResolution = RFResolution;

                IResearchModeSensorVLCFrame* pLFFrame = nullptr;
                winrt::check_hresult(pLFCameraFrame->QueryInterface(IID_PPV_ARGS(&pLFFrame)));
                IResearchModeSensorVLCFrame* pRFFrame = nullptr;
                winrt::check_hresult(pRFCameraFrame->QueryInterface(IID_PPV_ARGS(&pRFFrame)));

                size_t LFOutBufferCount = 0;
                const BYTE *pLFImage = nullptr;
                pLFFrame->GetBuffer(&pLFImage, &LFOutBufferCount);
                pHL2ResearchMode->m_LFbufferSize = LFOutBufferCount;
				size_t RFOutBufferCount = 0;
				const BYTE *pRFImage = nullptr;
				pRFFrame->GetBuffer(&pRFImage, &RFOutBufferCount);
				pHL2ResearchMode->m_RFbufferSize = RFOutBufferCount;

                // get tracking transform
                ResearchModeSensorTimestamp timestamp;
                pLFCameraFrame->GetTimeStamp(&timestamp);

                auto ts = PerceptionTimestampHelper::FromSystemRelativeTargetTime(HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
                auto transToWorld = pHL2ResearchMode->m_locator.TryLocateAtTimestamp(ts, pHL2ResearchMode->m_refFrame);
                if (transToWorld == nullptr)
                {
                    continue;
                }
                auto rot = transToWorld.Orientation();
                /*{
                    std::stringstream ss;
                    ss << rot.x << "," << rot.y << "," << rot.z << "," << rot.w << "\n";
                    std::string msg = ss.str();
                    std::wstring widemsg = std::wstring(msg.begin(), msg.end());
                    OutputDebugString(widemsg.c_str());
                }*/
                auto quatInDx = XMFLOAT4(rot.x, rot.y, rot.z, rot.w);
                auto rotMat = XMMatrixRotationQuaternion(XMLoadFloat4(&quatInDx));
                auto pos = transToWorld.Position();
                auto posMat = XMMatrixTranslation(pos.x, pos.y, pos.z);
                auto LfToWorld = pHL2ResearchMode->m_LFCameraPoseInvMatrix * rotMat * posMat;
				auto RfToWorld = pHL2ResearchMode->m_RFCameraPoseInvMatrix * rotMat * posMat;

                // save data
                {
                    std::lock_guard<std::mutex> l(pHL2ResearchMode->mu);

					// save LF and RF images
					if (!pHL2ResearchMode->m_LFImage)
					{
						OutputDebugString(L"Create Space for Left Front Image...\n");
						pHL2ResearchMode->m_LFImage = new UINT8[LFOutBufferCount];
					}
					memcpy(pHL2ResearchMode->m_LFImage, pLFImage, LFOutBufferCount * sizeof(UINT8));

					if (!pHL2ResearchMode->m_RFImage)
					{
						OutputDebugString(L"Create Space for Right Front Image...\n");
						pHL2ResearchMode->m_RFImage = new UINT8[RFOutBufferCount];
					}
					memcpy(pHL2ResearchMode->m_RFImage, pRFImage, RFOutBufferCount * sizeof(UINT8));
                }
				pHL2ResearchMode->m_LFImageUpdated = true;
				pHL2ResearchMode->m_RFImageUpdated = true;

                // release space
				if (pLFFrame) pLFFrame->Release();
				if (pRFFrame) pRFFrame->Release();

				if (pLFCameraFrame) pLFCameraFrame->Release();
				if (pRFCameraFrame) pRFCameraFrame->Release();
            }
        }
        catch (...) {}
        pHL2ResearchMode->m_LFSensor->CloseStream();
        pHL2ResearchMode->m_LFSensor->Release();
        pHL2ResearchMode->m_LFSensor = nullptr;

		pHL2ResearchMode->m_RFSensor->CloseStream();
		pHL2ResearchMode->m_RFSensor->Release();
		pHL2ResearchMode->m_RFSensor = nullptr;
    }

    void HL2ResearchMode::CamAccessOnComplete(ResearchModeSensorConsent consent)
    {
        camAccessCheck = consent;
        SetEvent(camConsentGiven);
    }

    inline UINT16 HL2ResearchMode::GetCenterDepth() {return m_centerDepth;}

    inline int HL2ResearchMode::GetDepthBufferSize() { return m_depthBufferSize; }

    inline bool HL2ResearchMode::DepthMapTextureUpdated() { return m_depthMapTextureUpdated; }

    inline bool HL2ResearchMode::ShortAbImageTextureUpdated() { return m_shortAbImageTextureUpdated; }

    inline bool HL2ResearchMode::PointCloudUpdated() { return m_pointCloudUpdated; }

    inline int HL2ResearchMode::GetLongDepthBufferSize() { return m_longDepthBufferSize; }

    inline bool HL2ResearchMode::LongDepthMapTextureUpdated() { return m_longDepthMapTextureUpdated; }

	inline bool HL2ResearchMode::LFImageUpdated() { return m_LFImageUpdated; }

	inline bool HL2ResearchMode::RFImageUpdated() { return m_RFImageUpdated; }

    // HI Addition
    inline bool HL2ResearchMode::ThreshAbImageTextureUpdated()
    {
        return m_threshAbImageTextureUpdated;
    }

    inline bool HL2ResearchMode::KeypointAbImageTextureUpdated()
    {
        return m_keypointAbImageTextureUpdated;
    }
    
    // not useful, remove this in next build!
    inline bool HL2ResearchMode::ToolDetectedByHoloLens()
    {
        return m_toolDetectedByHololens;
    }
    //
    hstring HL2ResearchMode::PrintDepthResolution()
    {
        std::string res_c_ctr = std::to_string(m_depthResolution.Height) + "x" + std::to_string(m_depthResolution.Width) + "x" + std::to_string(m_depthResolution.BytesPerPixel);
        return winrt::to_hstring(res_c_ctr);
    }

    hstring HL2ResearchMode::PrintDepthExtrinsics()
    {
        std::stringstream ss;
        ss << "Extrinsics: \n" << MatrixToString(m_depthCameraPose);
        std::string msg = ss.str();
        std::wstring widemsg = std::wstring(msg.begin(), msg.end());
        OutputDebugString(widemsg.c_str());
        return winrt::to_hstring(msg);
    }

	hstring HL2ResearchMode::PrintLFResolution()
	{
		std::string res_c_ctr = std::to_string(m_LFResolution.Height) + "x" + std::to_string(m_LFResolution.Width) + "x" + std::to_string(m_LFResolution.BytesPerPixel);
		return winrt::to_hstring(res_c_ctr);
	}

	hstring HL2ResearchMode::PrintLFExtrinsics()
	{
		std::stringstream ss;
		ss << "Extrinsics: \n" << MatrixToString(m_LFCameraPose);
		std::string msg = ss.str();
		std::wstring widemsg = std::wstring(msg.begin(), msg.end());
		OutputDebugString(widemsg.c_str());
		return winrt::to_hstring(msg);
	}

	hstring HL2ResearchMode::PrintRFResolution()
	{
		std::string res_c_ctr = std::to_string(m_RFResolution.Height) + "x" + std::to_string(m_RFResolution.Width) + "x" + std::to_string(m_RFResolution.BytesPerPixel);
		return winrt::to_hstring(res_c_ctr);
	}

	hstring HL2ResearchMode::PrintRFExtrinsics()
	{
		std::stringstream ss;
		ss << "Extrinsics: \n" << MatrixToString(m_RFCameraPose);
		std::string msg = ss.str();
		std::wstring widemsg = std::wstring(msg.begin(), msg.end());
		OutputDebugString(widemsg.c_str());
		return winrt::to_hstring(msg);
	}

    hstring HL2ResearchMode::GetDebugTrackedToolsetString()
    {
        std::stringstream ss;
        ss << "Full loop time " << m_debugLoopDuration << " ms";
        return winrt::to_hstring(ss.str());
    }

    void HL2ResearchMode::SetToolListByString(hstring const& tool_list)
    {
        PopulateToolList(to_string(tool_list), m_ToolList);
    }

    std::string HL2ResearchMode::MatrixToString(DirectX::XMFLOAT4X4 mat)
    {
        std::stringstream ss;
        for (size_t i = 0; i < 4; i++)
        {
            for (size_t j = 0; j < 4; j++)
            {
                ss << mat(i, j) << ",";
            }
            ss << "\n";
        }
        return ss.str();
    }
    
    // Stop the sensor loop and release buffer space.
    // Sensor object should be released at the end of the loop function
    void HL2ResearchMode::StopAllSensorDevice()
    {
        m_depthSensorLoopStarted = false;
        //m_pDepthUpdateThread->join();
        if (m_depthMap) 
        {
            delete[] m_depthMap;
            m_depthMap = nullptr;
        }
        if (m_depthMapTexture) 
        {
            delete[] m_depthMapTexture;
            m_depthMapTexture = nullptr;
        }
        if (m_pointCloud) 
        {
            m_pointcloudLength = 0;
            delete[] m_pointCloud;
            m_pointCloud = nullptr;
        }

        if (m_OutputToolPoseList) 
        {
            m_OutputToolPoseListLen = 0;
            delete[] m_OutputToolPoseList;
            m_OutputToolPoseList = nullptr;
        }

        m_longDepthSensorLoopStarted = false;
        if (m_longDepthMap)
        {
            delete[] m_longDepthMap;
            m_longDepthMap = nullptr;
        }
        if (m_longDepthMapTexture)
        {
            delete[] m_longDepthMapTexture;
            m_longDepthMapTexture = nullptr;
        }

        // HI Addition
        //

		m_pSensorDevice->Release();
		m_pSensorDevice = nullptr;
		m_pSensorDeviceConsent->Release();
		m_pSensorDeviceConsent = nullptr;
    }

    com_array<uint16_t> HL2ResearchMode::GetDepthMapBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_depthMap)
        {
            return com_array<uint16_t>();
        }
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_depthMap, m_depthMap + m_depthBufferSize);
        
        return tempBuffer;
    }

    com_array<uint16_t> HL2ResearchMode::GetShortAbImageBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_shortAbImage)
        {
            return com_array<uint16_t>();
        }
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_shortAbImage, m_shortAbImage + m_depthBufferSize);

        return tempBuffer;
    }

    // Get depth map texture buffer. (For visualization purpose)
    com_array<uint8_t> HL2ResearchMode::GetDepthMapTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_depthMapTexture) 
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_depthMapTexture), std::move_iterator(m_depthMapTexture + m_depthBufferSize));

        m_depthMapTextureUpdated = false;
        return tempBuffer;
    }

    // Get depth map texture buffer. (For visualization purpose)
    com_array<uint8_t> HL2ResearchMode::GetShortAbImageTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_shortAbImageTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_shortAbImageTexture), std::move_iterator(m_shortAbImageTexture + m_depthBufferSize));

        m_shortAbImageTextureUpdated = false;
        return tempBuffer;
    }

    com_array<uint16_t> HL2ResearchMode::GetLongDepthMapBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_longDepthMap)
        {
            return com_array<uint16_t>();
        }
        com_array<UINT16> tempBuffer = com_array<UINT16>(m_longDepthMap, m_longDepthMap + m_longDepthBufferSize);

        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetLongDepthMapTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_longDepthMapTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_longDepthMapTexture), std::move_iterator(m_longDepthMapTexture + m_longDepthBufferSize));

        m_longDepthMapTextureUpdated = false;
        return tempBuffer;
    }

	com_array<uint8_t> HL2ResearchMode::GetLFCameraBuffer()
	{
		std::lock_guard<std::mutex> l(mu);
		if (!m_LFImage)
		{
			return com_array<UINT8>();
		}
		com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_LFImage), std::move_iterator(m_LFImage + m_LFbufferSize));

		m_LFImageUpdated = false;
		return tempBuffer;
	}

	com_array<uint8_t> HL2ResearchMode::GetRFCameraBuffer()
	{
		std::lock_guard<std::mutex> l(mu);
		if (!m_RFImage)
		{
			return com_array<UINT8>();
		}
		com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_RFImage), std::move_iterator(m_RFImage + m_RFbufferSize));

		m_RFImageUpdated = false;
		return tempBuffer;
	}


    // Get the buffer for point cloud in the form of float array.
    // There will be 3n elements in the array where the 3i, 3i+1, 3i+2 element correspond to x, y, z component of the i'th point. (i->[0,n-1])
    com_array<float> HL2ResearchMode::GetPointCloudBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (m_pointcloudLength == 0)
        {
            return com_array<float>();
        }
        com_array<float> tempBuffer = com_array<float>(std::move_iterator(m_pointCloud), std::move_iterator(m_pointCloud + m_pointcloudLength));
        m_pointCloudUpdated = false;
        return tempBuffer;
    }

    // Get the 3D point (float[3]) of center point in depth map. Can be used to render depth cursor.
    com_array<float> HL2ResearchMode::GetCenterPoint()
    {
        std::lock_guard<std::mutex> l(mu);
        com_array<float> centerPoint = com_array<float>(std::move_iterator(m_centerPoint), std::move_iterator(m_centerPoint + 3));

        return centerPoint;
    }

    com_array<float> HL2ResearchMode::GetDepthSensorPosition()
    {
        std::lock_guard<std::mutex> l(mu);
        com_array<float> depthSensorPos = com_array<float>(std::move_iterator(m_depthSensorPosition), std::move_iterator(m_depthSensorPosition + 3));

        return depthSensorPos;
    }
    // !!!
    com_array<float> HL2ResearchMode::GetDepthToWorldMatrix()
    {
        std::lock_guard<std::mutex> l(mu);
        com_array<float> depthToWorldFloat = com_array<float>(std::move_iterator(m_depthToWorldMat), std::move_iterator(m_depthToWorldMat + 16));
        
        return depthToWorldFloat;
    }

    com_array<double> HL2ResearchMode::GetToolToWorldMatrix()
    {
        std::lock_guard<std::mutex> l(mu);
        com_array<double> toolToWorldFloat = com_array<double>(std::move_iterator(m_toolToWorldMat), std::move_iterator(m_toolToWorldMat + 16));
        return toolToWorldFloat;
    }

    com_array<double> HL2ResearchMode::GetTrackedToolsPoseMatrices()
    {
        std::lock_guard<std::mutex> l(mu);
        com_array<double> toolsToWorldDouble = com_array<double>(std::move_iterator(&(m_OutputToolPoseList[0])), std::move_iterator(&(m_OutputToolPoseList[0]) + m_OutputToolPoseListLen));
        m_toolDetectedByHololens = false; // flag is reset
        return toolsToWorldDouble;
    }

    // HI Additions
    com_array<uint8_t> HL2ResearchMode::GetShortThreshAbImageTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_shortThreshAbImageTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_shortThreshAbImageTexture), std::move_iterator(m_shortThreshAbImageTexture + m_depthBufferSize));

        m_threshAbImageTextureUpdated = false;
        return tempBuffer;
    }

    com_array<uint8_t> HL2ResearchMode::GetShortKeypointAbImageTextureBuffer()
    {
        std::lock_guard<std::mutex> l(mu);
        if (!m_shortKeypointAbImageTexture)
        {
            return com_array<UINT8>();
        }
        com_array<UINT8> tempBuffer = com_array<UINT8>(std::move_iterator(m_shortKeypointAbImageTexture), std::move_iterator(m_shortKeypointAbImageTexture + m_depthBufferSize));

        m_keypointAbImageTextureUpdated = false;
        return tempBuffer;
    }

    //-----

    // Set the reference coordinate system. Need to be set before the sensor loop starts; otherwise, default coordinate will be used.
    void HL2ResearchMode::SetReferenceCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem refCoord)
    {
        m_refFrame = refCoord;
    }

    void HL2ResearchMode::SetPointCloudRoiInSpace(float centerX, float centerY, float centerZ, float boundX, float boundY, float boundZ)
    {
        std::lock_guard<std::mutex> l(mu);

        m_useRoiFilter = true;
        m_roiCenter[0] = centerX;
        m_roiCenter[1] = centerY;
        m_roiCenter[2] = -centerZ;

        m_roiBound[0] = boundX;
        m_roiBound[1] = boundY;
        m_roiBound[2] = boundZ;
    }

    void HL2ResearchMode::SetPointCloudDepthOffset(uint16_t offset)
    {
        m_depthOffset = offset;
    }

    long long HL2ResearchMode::checkAndConvertUnsigned(UINT64 val)
    {
        assert(val <= kMaxLongLong);
        return static_cast<long long>(val);
    }

    // Extra Hisham Functions
    // These defines are used to distinguish between solutions
    // and relate to both the ACH and point touching accuracy
    #define ACH_ACCY_FOR_DISTANCE	0.0025	//2.5 mm, threshold for matching observed and expected point-sets
    #define ACH_ACCY_FOR_REG		0.00000225	//1.5 mm RMS error
    #define ACH_ACCY_FOR_DUPLICATES	0.001	//1 mm

    // util function for string splitting
    static std::vector<std::string> split(const std::string& s, char delimiter)
    {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }

    void HL2ResearchMode::PopulateToolList(std::string encoded_String, std::vector<TrackedTool>& toolList)
    {
        // units for tool triplets is metres!
        // expected structure: [1,x1,y1,z1,...;2,x1,y1,z1,...] i.e. semi colon delimited
        // 'tools' which start with and integer ID and are followed by a series of triplets
        // no expectation for ID ints in tool packet to be ordered

        std::vector<std::string> delimitedMsg = split(encoded_String, ';');

        std::string toolTripletsString;
        std::vector<std::string> toolTripletsStr;
        std::string id;
        for (std::string toolSubstring : delimitedMsg)
        {
            if (toolSubstring.size() == 0) continue;
            TrackedTool tool;
            toolTripletsStr = split(toolSubstring, ',');

            id = toolTripletsStr[0];
            tool.ID = std::stoi(id);

            for (size_t i = 0; i < (toolTripletsStr.size() - 1) / 3; i++)
            {
                tool.GeometryPoints.push_back(Eigen::Vector3d(
                    std::stod(toolTripletsStr[i * 3 + 1]),
                    std::stod(toolTripletsStr[i * 3 + 2]),
                    std::stod(toolTripletsStr[i * 3 + 3])));
            }

            toolList.push_back(tool);
        }

        if (toolList.size() == 0) return;
        if (!m_OutputToolPoseList)
        {
            m_OutputToolPoseList = new double[toolList.size() * 18];
            m_OutputToolPoseListLen = toolList.size() * 18;
        }
    }
    
    void HL2ResearchMode::TryLocatingTools(std::vector<InfraredKeypoint> infraredBlobs, std::vector<TrackedTool>& tools)
    {
        // un-ordered lists of marker 3-D positions/image pixel locations
        std::vector<Eigen::Vector3d> collectedPoints;
        std::vector<Eigen::Vector3d> collectedDepthPoints;
        std::vector<cv::KeyPoint> collectedKeyPts;

        for (const InfraredKeypoint& kp : infraredBlobs)
        {
            collectedPoints.push_back(kp.WorldLocation);
            collectedKeyPts.push_back(kp.ImageKeypoint);
            collectedDepthPoints.push_back(kp.DepthLocation);
        }

        TrackedTool tool;
        for (size_t i = 0; i < tools.size(); i++)
        {
            // zero/clear all properties as necessary!
            tool = tools[i];
            tool.PoseMatrix_HoloWorld = Eigen::Matrix4d::Identity();
            tool.VisibleToHoloLens = false;
            
            tool.ObservedPoints_World.clear();
            tool.ObservedImgKeypoints.clear();
            tool.ObservedPoints_Depth.clear();

            std::vector<std::vector<int>> candidateList; // will be updated to order point sets

            // check if the TrackedTool tool is visible in this frame, if it isn't, then we assume it's not visible to the sensor
            if (!GetPointCorrespondence(tool.GeometryPoints, collectedPoints, candidateList)) { tools[i] = tool; continue; }
            
            else
            { // tracked tool is visible, and GetPointCorrespondence has updated candidateList
                if (candidateList.size() != 0)
                {
                    int index = 0;

                    // ordered list of indices from larger pointset
                    std::vector<int> indexList = candidateList[0];

                    for (size_t j = 0; j < indexList.size(); j++)
                    {
                        index = indexList[j];
                        if (index > -1 && index < collectedPoints.size())
                        {
                            tool.ObservedPoints_World.push_back(collectedPoints[index]);
                            tool.ObservedImgKeypoints.push_back(collectedKeyPts[index]);
                            tool.ObservedPoints_Depth.push_back(collectedDepthPoints[index]);
                        }
                    }

                    if (tool.GeometryPoints.size() == tool.ObservedPoints_World.size()) // safety check
                    {
                        tool.PoseMatrix_HoloWorld = ComputeRigidTransform(tool.GeometryPoints, tool.ObservedPoints_World); // SVD to get transformation of tool to holographic world frame
                        tool.PoseMatrix_HoloWorld.transposeInPlace(); // for storage as row major
                        tool.VisibleToHoloLens = true;

                        // Remove points associated with a tool that was found to reduce search size for next loop
                        // sort indices smallest to largest
                        std::sort(indexList.begin(), indexList.end());

                        // iterate backwards to preserve order
                        for (size_t k = indexList.size() - 1; k != -1; k--)
                        {
                            infraredBlobs.erase(infraredBlobs.begin() + indexList[k]);
                            collectedPoints.erase(collectedPoints.begin() + indexList[k]);
                            collectedKeyPts.erase(collectedKeyPts.begin() + indexList[k]);
                            collectedDepthPoints.erase(collectedDepthPoints.begin() + indexList[k]);
                        }
                    }

                }
            }

            tools[i] = tool;

        }
    }

    void HL2ResearchMode::GetFormattedDoubleMat(std::vector<TrackedTool> tools, double* outputDoubleArr)
    {
        // formats each tool into a double array
        // transform matrix stored as row major
        // [toolID,toolVisible(0/1),mat00,mat01,mat02...,mat33]
        for (size_t i = 0; i < tools.size(); i++)
        {
            outputDoubleArr[i * 18] = tools[i].ID; // tool ID stored
            outputDoubleArr[i * 18 + 1] = tools[i].VisibleToHoloLens ? 1 : 0; // tool visibility stored

            std::copy(tools[i].PoseMatrix_HoloWorld.data(), tools[i].PoseMatrix_HoloWorld.data() + 16, outputDoubleArr + i * 18 + 2); // tool pose matrix stored
        }
    }
    
    // Converts std::vector<Eigen::Vector3d> to std::vector<XMVECTOR> 
    void HL2ResearchMode::Eigen_Vec2DX_Vec(std::vector<Eigen::Vector3d>& Input_CV_Vec, std::vector<DirectX::XMVECTOR>& Output_DX_Vec)
    {
        for (size_t i = 0; i != Input_CV_Vec.size(); i++)
        {
            const XMFLOAT3 tempInput((float)Input_CV_Vec[i].x(), (float)Input_CV_Vec[i].y(), (float)Input_CV_Vec[i].z());
            Output_DX_Vec.push_back(XMLoadFloat3(&tempInput));
        }
    }

    //---------------------------------------------------------------------------------------------------------------
    // Helper functions for ordering + registering two point sets, reusing old legacy code from another project, fairly optimised, but could definitely
    // do with better documenting + renaming
    
    bool HL2ResearchMode::GetPointCorrespondence(std::vector<Eigen::Vector3d> ReferencePoints,
        std::vector<Eigen::Vector3d> CollectedPoints,
        std::vector<std::vector<int>>& CorrespondenceList)
    {
        std::vector<std::vector<int>> theCandidateList;
        //std::vector<DirectX::XMVECTOR>::iterator pInputHole;
        std::vector<Eigen::Vector3d>::iterator pInputHole;
        std::vector<int>::iterator pCandidate;

        Eigen::Vector3d theDifference;
        float theDifferenceMagnitude;

        // Make sure only one position per hole is in the list
        PruneDuplicates(ReferencePoints);
        PruneDuplicates(CollectedPoints);

        // Need at least three points
        if (ReferencePoints.size() < 3 || CollectedPoints.size() < 3)
            return false;

        // Add first point to list
        GenerateCandidatesList(theCandidateList, CollectedPoints.size());

        for (pInputHole = (ReferencePoints.begin() + 1); pInputHole != ReferencePoints.end(); pInputHole++)
        {
            // Add another point to the list
            GenerateCandidatesList(theCandidateList, CollectedPoints.size());

            // difference between two consecutive points
            theDifference = (*(pInputHole)) - (*(pInputHole - 1));

            theDifferenceMagnitude = theDifference.norm();

            // Prune options depending on distance between first two points
            PruneByDistance(theDifferenceMagnitude, theCandidateList, &CollectedPoints[0]);
        }

        CorrespondenceList = theCandidateList;
        return true;
    }

    bool HL2ResearchMode::PruneByDistance(float inDistance, std::vector<std::vector<int>>& inOptionList, Eigen::Vector3d* unregisteredPoints)
    {
        std::vector<std::vector<int> >::iterator pOptionList;
        std::vector<std::vector<int> >::iterator temp;

        Eigen::Vector3d theVectorDifference;

        for (pOptionList = inOptionList.begin(); pOptionList != inOptionList.end(); )
        {
            // Need at least two points per possible list for distance pruning
            if ((*pOptionList).size() < 2)
                return false;

            theVectorDifference = unregisteredPoints[*((*pOptionList).end() - 1)] - unregisteredPoints[*((*pOptionList).end() - 2)];

            float diffMag = fabs(theVectorDifference.norm() - inDistance);
            if (diffMag > ACH_ACCY_FOR_DISTANCE)	// if not equal
            {
                pOptionList = inOptionList.erase(pOptionList);
            }
            else
            {
                ++pOptionList;
            }
        }

        return true;
    }

    bool HL2ResearchMode::PruneByDistance(float inDistance, std::vector<std::vector<int> >& inOptionList, XMVECTOR* unregisteredPoints)
    {
        std::vector<std::vector<int> >::iterator pOptionList;
        std::vector<std::vector<int> >::iterator temp;

        DirectX::XMVECTOR theVectorDifference;

        for (pOptionList = inOptionList.begin(); pOptionList != inOptionList.end(); )
        {
            // Need at least two points per possible list for distance pruning
            if ((*pOptionList).size() < 2)
                return false;

            theVectorDifference = unregisteredPoints[*((*pOptionList).end() - 1)] - unregisteredPoints[*((*pOptionList).end() - 2)];

            float diffMag = fabs(GetVectorLength(theVectorDifference) - inDistance);
            if (diffMag > ACH_ACCY_FOR_DISTANCE)	// if equal
            {
                pOptionList = inOptionList.erase(pOptionList);
            }
            else
            {
                ++pOptionList;
            }
        }

        return true;
    }

    bool HL2ResearchMode::PruneDuplicates(std::vector<Eigen::Vector3d>& ioPointList)
    {
        std::vector<Eigen::Vector3d>::iterator pPoint, pPoint2;
        Eigen::Vector3d theVectorDifference;

        // Clear up any duplicates
        for (pPoint = ioPointList.begin(); pPoint != ioPointList.end(); pPoint++)
        {
            for (pPoint2 = (pPoint + 1); pPoint2 != ioPointList.end(); pPoint2++)
            {
                theVectorDifference = (*pPoint) - (*pPoint2);

                if (theVectorDifference.norm() < ACH_ACCY_FOR_DUPLICATES)
                {
                    ioPointList.erase(pPoint2);
                    pPoint2--;
                }
            }
        }

        return true;
    }

    bool HL2ResearchMode::PruneDuplicates(std::vector<DirectX::XMVECTOR>& ioPointList)
    {
        std::vector<DirectX::XMVECTOR>::iterator pPoint, pPoint2;
        DirectX::XMVECTOR theVectorDifference;

        // Clear up any duplicates
        for (pPoint = ioPointList.begin(); pPoint != ioPointList.end(); pPoint++)
        {
            for (pPoint2 = (pPoint + 1); pPoint2 != ioPointList.end(); pPoint2++)
            {
                theVectorDifference = (*pPoint) - (*pPoint2);

                if (GetVectorLength(theVectorDifference) < ACH_ACCY_FOR_DUPLICATES)
                {
                    ioPointList.erase(pPoint2);
                    pPoint2--;
                }
            }
        }

        return true;
    }

    float HL2ResearchMode::GetVectorLength(DirectX::XMVECTOR xmVec)
    {
        XMFLOAT3 vecLenVec;
        DirectX::XMStoreFloat3(&vecLenVec, DirectX::XMVector3Length(xmVec));
        float magnitude = vecLenVec.x;

        return magnitude;
    }

    bool HL2ResearchMode::GenerateCandidatesList(std::vector<std::vector<int> >& inOptionList, int unregPointsCount)
    {
        std::vector<std::vector<int> >::iterator pOptionList;
        std::vector<int>::iterator pOptionElement;
        std::vector<int> theTempOption;
        std::vector<std::vector<int> > theOldOptionList;

        // If input list is empty create a 1 column list of possible points (possibilities = number
        // of holes in clamp)
        if (inOptionList.size() == 0)
        {
            for (int i = 0; i < unregPointsCount; i++)
            {
                theTempOption.clear();
                theTempOption.push_back(i);
                inOptionList.push_back(theTempOption);
            }

            return true;
        }

        theOldOptionList = inOptionList;

        inOptionList.clear();

        for (pOptionList = theOldOptionList.begin(); pOptionList != theOldOptionList.end(); pOptionList++)
        {
            for (int i = 0; i < unregPointsCount; i++)
            {
                bool found = false;

                for (pOptionElement = (*pOptionList).begin(); pOptionElement != (*pOptionList).end(); pOptionElement++)
                {
                    if ((*pOptionElement) == i)
                        found = true;
                }

                if (!found)
                {
                    theTempOption = (*pOptionList);
                    theTempOption.push_back((i));
                    inOptionList.push_back(theTempOption);
                }
            }
        }

        // In this case, no more combinations can be produced from the hole points
        // available, which means registration cannot be completed successfully
        // (e.g. for a perfectly symmetrical set of points).
        if (inOptionList.size() == 0)
        {
            return false;
        }

        return true;
    }

}
