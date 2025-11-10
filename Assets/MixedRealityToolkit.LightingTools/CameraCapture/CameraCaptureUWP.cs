// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License. See LICENSE in the project root for license information.
#if WINDOWS_UWP
using System;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;
using Unity.Collections;
using UnityEngine;
using UnityEngine.XR;
using Windows.Graphics.Imaging;
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Media.Devices;
using Windows.Media.MediaProperties;
using Windows.Storage.Streams;

namespace Microsoft.MixedReality.Toolkit.LightingTools
{
    /// <summary>
    /// An <see cref="ICameraCapture"/> service that works with the Windows.MediaCapture APIs used by OpenXR.
    /// </summary>
    public class CameraCaptureUWP : ICameraCapture, ICameraControl
    {
        #region Member Variables
        private readonly object requestLock = new object();

        private Texture2D cacheTex = null;
        private Texture2D resizedTex = null;

        private CameraResolution resolution = null;
        private float fieldOfView = 45.0f;

        private MediaCapture mediaCapture = null;
        private MediaFrameReader frameReader = null;
        private Task initializeTask = null;

        private TaskCompletionSource<TextureResult> pendingTextureRequest = null;
        private TaskCompletionSource<ColorResult> pendingColorRequest = null;

        private double exposure = 0.2f;
        private uint iso = 800;
        private uint temperature = 5000;

        private bool isReady = false;
        #endregion // Member Variables

        #region Internal Methods
        /// <summary>
        /// Ensures that the camera system has been fully initialized, otherwise throws an exception.
        /// </summary>
        private void EnsureInitialized()
        {
            if (!IsReady)
            {
                throw new InvalidOperationException($"{nameof(InitializeAsync)} must be completed first.");
            }
        }

        /// <summary>
        /// Internal version of initialize. Should not be called more than once unless Shutdown has been called.
        /// </summary>
        /// <param name="preferGPUTexture">Ignored in the new pipeline as textures are always generated on the GPU.</param>
        /// <param name="preferredResolution">Preferred resolution for taking pictures.</param>
        /// <returns>A <see cref="Task"/> that represents the operation.</returns>
        private async Task InnerInitializeAsync(bool preferGPUTexture, CameraResolution preferredResolution)
        {
            resolution = preferredResolution ?? new CameraResolution();

            mediaCapture = new MediaCapture();
            var settings = new MediaCaptureInitializationSettings
            {
                StreamingCaptureMode = StreamingCaptureMode.Video,
                MemoryPreference = MediaCaptureMemoryPreference.Cpu
            };
            await mediaCapture.InitializeAsync(settings).AsTask().ConfigureAwait(false);

            MediaFrameSource frameSource = mediaCapture.FrameSources.Values
                .FirstOrDefault(source => source.Info.SourceKind == MediaFrameSourceKind.Color);

            if (frameSource == null)
            {
                throw new InvalidOperationException("Unable to locate a usable color camera frame source.");
            }

            MediaFrameFormat format = frameSource.SupportedFormats
                .Where(f => f.VideoFormat != null && f.VideoFormat.Width > 0 && f.VideoFormat.Height > 0)
                .OrderByDescending(f => (long)f.VideoFormat.Width * f.VideoFormat.Height)
                .FirstOrDefault(f => string.Equals(f.Subtype, MediaEncodingSubtypes.Bgra8, StringComparison.OrdinalIgnoreCase));

            if (format != null)
            {
                await frameSource.SetFormatAsync(format).AsTask().ConfigureAwait(false);
            }

            frameReader = await mediaCapture.CreateFrameReaderAsync(frameSource, MediaEncodingSubtypes.Bgra8).AsTask().ConfigureAwait(false);
            frameReader.AcquisitionMode = MediaFrameReaderAcquisitionMode.Realtime;
            frameReader.FrameArrived += FrameReader_FrameArrived;

            MediaFrameReaderStartStatus startStatus = await frameReader.StartAsync().AsTask().ConfigureAwait(false);
            if (startStatus != MediaFrameReaderStartStatus.Success)
            {
                throw new InvalidOperationException($"Unable to start frame reader. Status: {startStatus}.");
            }

            await ApplyInitialCameraSettingsAsync().ConfigureAwait(false);

            isReady = true;
        }

        /// <summary>
        /// Applies the initial exposure, white balance, and ISO values to the camera.
        /// </summary>
        private async Task ApplyInitialCameraSettingsAsync()
        {
            var exposureControl = mediaCapture.VideoDeviceController?.ExposureControl;
            if (exposureControl != null && exposureControl.Supported)
            {
                if (exposureControl.Auto)
                {
                    await exposureControl.SetAutoAsync(false).AsTask().ConfigureAwait(false);
                }

                TimeSpan min = exposureControl.Min;
                TimeSpan max = exposureControl.Max;
                double clamped = Mathf.Clamp01((float)exposure);
                long ticks = (long)((max - min).Ticks * clamped);
                await exposureControl.SetValueAsync(min + TimeSpan.FromTicks(ticks)).AsTask().ConfigureAwait(false);
            }

            var whiteBalanceControl = mediaCapture.VideoDeviceController?.WhiteBalanceControl;
            if (whiteBalanceControl != null && whiteBalanceControl.Supported)
            {
                uint value = ClampToRange(temperature, whiteBalanceControl.Min, whiteBalanceControl.Max);
                await whiteBalanceControl.SetValueAsync(value).AsTask().ConfigureAwait(false);
            }

            var isoControl = mediaCapture.VideoDeviceController?.IsoSpeedControl;
            if (isoControl != null && isoControl.Supported)
            {
                uint value = ClampToRange(iso, isoControl.Min, isoControl.Max, isoControl.Step);
                await isoControl.SetValueAsync(value).AsTask().ConfigureAwait(false);
            }
        }

        private static uint ClampToRange(uint value, uint min, uint max, uint step = 1)
        {
            uint clamped = Math.Max(min, Math.Min(max, value));
            if (step <= 1)
            {
                return clamped;
            }

            uint delta = (clamped - min) / step;
            return min + delta * step;
        }

        /// <summary>
        /// Handles the frame arrived event from the frame reader.
        /// </summary>
        private async void FrameReader_FrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args)
        {
            if (!IsRequestingImage)
            {
                return;
            }

            using (MediaFrameReference frame = sender.TryAcquireLatestFrame())
            {
                var videoFrame = frame?.VideoMediaFrame;
                if (videoFrame == null)
                {
                    return;
                }

                var softwareBitmap = videoFrame.SoftwareBitmap;
                if (softwareBitmap == null)
                {
                    return;
                }

                using (softwareBitmap)
                using (SoftwareBitmap bitmap = ConvertToBgra(softwareBitmap))
                {
                    float? calculatedFov = TryCalculateFieldOfView(videoFrame);
                    if (calculatedFov.HasValue)
                    {
                        fieldOfView = calculatedFov.Value;
                    }

                    int width = bitmap.PixelWidth;
                    int height = bitmap.PixelHeight;
                    byte[] data = new byte[width * height * 4];
                    bitmap.CopyToBuffer(data.AsBuffer());

                    Matrix4x4 matrix = GetHeadPoseMatrix();

                    await DeliverFrameOnAppThreadAsync(data, width, height, matrix).ConfigureAwait(false);
                }
            }
        }

        private static SoftwareBitmap ConvertToBgra(SoftwareBitmap source)
        {
            if (source.BitmapPixelFormat == BitmapPixelFormat.Bgra8 && source.BitmapAlphaMode == BitmapAlphaMode.Premultiplied)
            {
                return SoftwareBitmap.Copy(source);
            }

            return SoftwareBitmap.Convert(source, BitmapPixelFormat.Bgra8, BitmapAlphaMode.Premultiplied);
        }

        private float? TryCalculateFieldOfView(VideoMediaFrame frame)
        {
            var intrinsics = frame.CameraIntrinsics;
            if (intrinsics == null)
            {
                return null;
            }

            double focalLengthY = intrinsics.FocalLength.Y;
            double imageHeight = intrinsics.ImageHeight;
            if (focalLengthY <= 0 || imageHeight <= 0)
            {
                return null;
            }

            double verticalFov = 2.0 * Math.Atan(imageHeight / (2.0 * focalLengthY));
            return (float)(verticalFov * Mathf.Rad2Deg);
        }

        private Matrix4x4 GetHeadPoseMatrix()
        {
            if (Camera.main != null)
            {
                return Camera.main.transform.localToWorldMatrix;
            }

            Vector3 position = Vector3.zero;
            Quaternion rotation = Quaternion.identity;

            try
            {
                if (XRSettings.isDeviceActive)
                {
                    position = InputTracking.GetLocalPosition(XRNode.CenterEye);
                    rotation = InputTracking.GetLocalRotation(XRNode.CenterEye);
                }
            }
            catch
            {
                // XR system not available - fall back to identity transform.
            }

            return Matrix4x4.TRS(position, rotation, Vector3.one);
        }

        private Task DeliverFrameOnAppThreadAsync(byte[] data, int width, int height, Matrix4x4 matrix)
        {
            var tcs = new TaskCompletionSource<bool>();
            UnityEngine.WSA.Application.InvokeOnAppThread(() =>
            {
                try
                {
                    DeliverFrame(data, width, height, matrix);
                    tcs.SetResult(true);
                }
                catch (Exception ex)
                {
                    tcs.SetException(ex);
                }
            }, false);
            return tcs.Task;
        }

        private void DeliverFrame(byte[] data, int width, int height, Matrix4x4 matrix)
        {
            if (cacheTex == null || cacheTex.width != width || cacheTex.height != height)
            {
                if (cacheTex != null)
                {
                    UnityEngine.Object.Destroy(cacheTex);
                }

                cacheTex = new Texture2D(width, height, TextureFormat.BGRA32, false);
                cacheTex.wrapMode = TextureWrapMode.Clamp;
            }

            cacheTex.LoadRawTextureData(data);
            cacheTex.Apply(false);

            Texture texture = cacheTex;
            if (resolution != null)
            {
                resolution.ResizeTexture(cacheTex, ref texture, true);
                resizedTex = texture as Texture2D;
            }
            else
            {
                resizedTex = cacheTex;
            }

            var textureResult = new TextureResult(matrix, resizedTex);

            pendingTextureRequest?.TrySetResult(textureResult);
            pendingTextureRequest = null;

            if (pendingColorRequest != null)
            {
                NativeArray<Color24> colors = resizedTex.GetRawTextureData<Color24>();
                pendingColorRequest.TrySetResult(new ColorResult(matrix, resizedTex, colors));
                pendingColorRequest = null;
            }
        }
        #endregion // Internal Methods

        #region Public Methods
        /// <inheritdoc/>
        public Task InitializeAsync(bool preferGPUTexture, CameraResolution preferredResolution)
        {
            if (initializeTask != null)
            {
                throw new InvalidOperationException("Already initializing.");
            }

            initializeTask = InnerInitializeAsync(preferGPUTexture, preferredResolution);
            return initializeTask;
        }

        /// <inheritdoc/>
        public Task<ColorResult> RequestColorAsync()
        {
            EnsureInitialized();

            lock (requestLock)
            {
                if (IsRequestingImage)
                {
                    throw new InvalidOperationException("Already requesting an image.");
                }

                pendingColorRequest = new TaskCompletionSource<ColorResult>();
                return pendingColorRequest.Task;
            }
        }

        /// <inheritdoc/>
        public Task<TextureResult> RequestTextureAsync()
        {
            EnsureInitialized();

            lock (requestLock)
            {
                if (IsRequestingImage)
                {
                    throw new InvalidOperationException("Already requesting an image.");
                }

                pendingTextureRequest = new TaskCompletionSource<TextureResult>();
                return pendingTextureRequest.Task;
            }
        }

        /// <inheritdoc/>
        public async Task SetExposureAsync(double exposure)
        {
            EnsureInitialized();

            this.exposure = exposure;
            var exposureControl = mediaCapture.VideoDeviceController?.ExposureControl;
            if (exposureControl == null || !exposureControl.Supported)
            {
                return;
            }

            if (exposureControl.Auto)
            {
                await exposureControl.SetAutoAsync(false).AsTask().ConfigureAwait(false);
            }

            TimeSpan min = exposureControl.Min;
            TimeSpan max = exposureControl.Max;
            double clamped = Mathf.Clamp01((float)exposure);
            long ticks = (long)((max - min).Ticks * clamped);
            await exposureControl.SetValueAsync(min + TimeSpan.FromTicks(ticks)).AsTask().ConfigureAwait(false);
        }

        /// <inheritdoc/>
        public async Task SetWhiteBalanceAsync(uint temperature)
        {
            EnsureInitialized();

            this.temperature = temperature;
            var control = mediaCapture.VideoDeviceController?.WhiteBalanceControl;
            if (control == null || !control.Supported)
            {
                return;
            }

            uint value = ClampToRange(temperature, control.Min, control.Max);
            await control.SetValueAsync(value).AsTask().ConfigureAwait(false);
        }

        /// <inheritdoc/>
        public async Task SetISOAsync(uint iso)
        {
            EnsureInitialized();

            this.iso = iso;
            var control = mediaCapture.VideoDeviceController?.IsoSpeedControl;
            if (control == null || !control.Supported)
            {
                return;
            }

            uint value = ClampToRange(iso, control.Min, control.Max, control.Step);
            await control.SetValueAsync(value).AsTask().ConfigureAwait(false);
        }

        /// <inheritdoc/>
        public void Shutdown()
        {
            isReady = false;

            if (frameReader != null)
            {
                frameReader.FrameArrived -= FrameReader_FrameArrived;
                var _ = frameReader.StopAsync();
                frameReader.Dispose();
                frameReader = null;
            }

            if (mediaCapture != null)
            {
                mediaCapture.Dispose();
                mediaCapture = null;
            }

            if (cacheTex != null)
            {
                UnityEngine.Object.Destroy(cacheTex);
                cacheTex = null;
            }

            if (resizedTex != null && resizedTex != cacheTex)
            {
                UnityEngine.Object.Destroy(resizedTex);
                resizedTex = null;
            }

            pendingTextureRequest = null;
            pendingColorRequest = null;
            initializeTask = null;
        }
        #endregion // Public Methods

        #region Public Properties
        /// <inheritdoc/>
        public bool IsReady => isReady && initializeTask != null && initializeTask.IsCompleted;

        /// <inheritdoc/>
        public bool IsRequestingImage => pendingTextureRequest != null || pendingColorRequest != null;

        /// <inheritdoc/>
        public float FieldOfView => fieldOfView;
        #endregion // Public Properties
    }
}
#endif // WINDOWS_UWP
