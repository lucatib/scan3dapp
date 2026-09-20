// This file exists ONLY to pin, by compilation, the exact C# API surface that the
// Vapolia.Google.ARCore (1.47.1) binding exposes for the native ARCore SDK. None of
// these methods are ever called at runtime — the class is never referenced from
// MauiProgram, MainActivity, or any page. Its sole purpose is to prove that every
// ARCore call the scanner will need (session lifecycle, depth images, camera pose,
// intrinsics, coordinate transforms) compiles cleanly against this binding.
//
// See docs/arcore-binding-notes.md for the evaluation of candidate bindings and for
// a copy of every signature pinned here.

using Android.App;
using Android.Content;
using Google.AR.Core;
using Google.AR.Core.Exceptions;
using Java.Nio;
// Both Microsoft.Maui.Controls (implicitly usable via global usings in the MAUI SDK)
// and the ARCore binding define a type called "Frame", and both Microsoft.Maui.Controls
// and Android.Media define a type called "Image". Alias the ARCore/Android ones
// explicitly to avoid CS0104 ambiguous-reference errors.
using ArFrame = Google.AR.Core.Frame;
using ArPlane = Google.AR.Core.Plane;
using AndroidImage = Android.Media.Image;

namespace Scanner.App.Platforms.Android;

/// <summary>
/// Compile-only probe. Do not call from application code; do not delete without
/// re-verifying the binding still exposes these members.
/// </summary>
internal static class ArCoreApiProbe
{
    // ---- 1. Availability check + APK install request --------------------------------

    internal static void CheckAvailabilityAndRequestInstall(Context context, Activity activity)
    {
        ArCoreApk.Availability availability = ArCoreApk.Instance!.CheckAvailability(context)!;
        bool isSupported = availability.IsSupported;
        bool isTransient = availability.IsTransient;
        bool isUnsupported = availability.IsUnsupported;
        _ = (isSupported, isTransient, isUnsupported);

        ArCoreApk.InstallStatus status = ArCoreApk.Instance!.RequestInstall(activity, true)!;
        _ = status;
    }

    // ---- 2. Session + Config lifecycle, depth mode, focus mode -----------------------

    internal static Session CreateAndConfigureSession(Context context)
    {
        var session = new Session(context);
        var config = new Config(session);

        bool depthSupported = session.IsDepthModeSupported(Config.DepthMode.Automatic!);
        config.SetDepthMode(depthSupported ? Config.DepthMode.Automatic! : Config.DepthMode.Disabled!);
        config.SetFocusMode(Config.FocusMode.Auto!);
        config.SetPlaneFindingMode(Config.PlaneFindingMode.Horizontal!);

        session.Configure(config);
        session.Resume();
        session.Pause();
        session.Close();

        return session;
    }

    // ---- 3. Per-frame update plumbing --------------------------------------------------

    internal static ArFrame UpdateFrame(Session session, int cameraTextureId, int displayRotation, int width, int height)
    {
        session.SetCameraTextureName(cameraTextureId);
        session.SetDisplayGeometry(displayRotation, width, height);
        ArFrame frame = session.Update()!;

        long frameTimestamp = frame.Timestamp;
        bool displayGeometryChanged = frame.HasDisplayGeometryChanged;
        _ = (frameTimestamp, displayGeometryChanged);

        return frame;
    }

    // ---- 4. Tracking state, camera pose, view/projection matrices ---------------------

    internal static void ReadCameraPose(ArFrame frame)
    {
        Camera camera = frame.Camera!;
        TrackingState trackingState = camera.TrackingState!;

        if (trackingState.Equals(TrackingState.Tracking!))
        {
            var poseMatrix = new float[16];
            camera.Pose!.ToMatrix(poseMatrix, 0);

            var viewMatrix = new float[16];
            camera.GetViewMatrix(viewMatrix, 0);

            var projectionMatrix = new float[16];
            camera.GetProjectionMatrix(projectionMatrix, 0, 0.1f, 100f);

            _ = (poseMatrix, viewMatrix, projectionMatrix);
        }
    }

    // ---- 5. Depth images (DEPTH16), raw depth, raw depth confidence -------------------

    internal static void ReadDepthImages(ArFrame frame)
    {
        try
        {
            using AndroidImage depth = frame.AcquireDepthImage16Bits()!;
            ReadImagePlane(depth);
        }
        catch (NotYetAvailableException)
        {
            // The depth image for this frame is not ready yet; caller should retry
            // on the next Session.Update() call.
        }

        try
        {
            using AndroidImage rawDepth = frame.AcquireRawDepthImage16Bits()!;
            ReadImagePlane(rawDepth);
        }
        catch (NotYetAvailableException)
        {
        }

        try
        {
            using AndroidImage confidence = frame.AcquireRawDepthConfidenceImage()!;
            ReadImagePlane(confidence);
        }
        catch (NotYetAvailableException)
        {
        }
    }

    private static void ReadImagePlane(AndroidImage image)
    {
        int width = image.Width;
        int height = image.Height;
        long timestamp = image.Timestamp;

        AndroidImage.Plane[] planes = image.GetPlanes()!;
        AndroidImage.Plane plane0 = planes[0]!;
        ByteBuffer buffer = plane0.Buffer!;
        int rowStride = plane0.RowStride;
        int pixelStride = plane0.PixelStride;

        _ = (width, height, timestamp, buffer, rowStride, pixelStride);
    }

    // ---- 6. Camera intrinsics (focal length, principal point, image dimensions) ------

    internal static void ReadIntrinsics(Camera camera)
    {
        CameraIntrinsics textureIntrinsics = camera.TextureIntrinsics!;
        CameraIntrinsics imageIntrinsics = camera.ImageIntrinsics!;

        float[] focalLength = imageIntrinsics.GetFocalLength()!;
        float[] principalPoint = imageIntrinsics.GetPrincipalPoint()!;
        int[] imageDimensions = imageIntrinsics.GetImageDimensions()!;

        _ = (textureIntrinsics, focalLength, principalPoint, imageDimensions);
    }

    // ---- 7. Camera-background texture coordinate transform ---------------------------

    internal static float[] TransformTextureCoordinates(ArFrame frame, float[] normalizedQuadCoords)
    {
        var transformed = new float[normalizedQuadCoords.Length];
        frame.TransformCoordinates2d(
            Coordinates2d.ViewNormalized!,
            normalizedQuadCoords,
            Coordinates2d.TextureNormalized!,
            transformed);

        // The camera background draws a clip-space quad, so it maps from OpenGL NDC rather than view coordinates.
        frame.TransformCoordinates2d(
            Coordinates2d.OpenglNormalizedDeviceCoordinates!,
            normalizedQuadCoords,
            Coordinates2d.TextureNormalized!,
            transformed);
        return transformed;
    }

    // ---- 8. Hit test under the crosshair, trackable planes, pose translation ----------

    internal static void PickTargetAndSupportPlane(Session session, ArFrame frame, float x, float y)
    {
        HitResult? hit = frame.HitTest(x, y)?.FirstOrDefault();
        if (hit is null) return;

        Pose hitPose = hit.HitPose!;
        float tx = hitPose.Tx();
        float ty = hitPose.Ty();
        float tz = hitPose.Tz();
        _ = (tx, ty, tz);

        foreach (ITrackable trackable in session.GetAllTrackables(Java.Lang.Class.FromType(typeof(ArPlane))!)!)
        {
            if (trackable is not ArPlane plane) continue;

            ArPlane? subsumedBy = plane.SubsumedBy;
            Pose centerPose = plane.CenterPose!;
            // The binding exposes Java getType() as a GetType() that hides object.GetType(); the typed local pins
            // that this member returns ARCore's Plane.Type and not System.Type.
            ArPlane.Type planeType = plane.GetType()!;

            _ = (subsumedBy, centerPose.Ty(), planeType.Equals(ArPlane.Type.HorizontalUpwardFacing!));
        }
    }
}
