using System.Numerics;
using Foundation;
using Microsoft.Maui.Handlers;
using ModelIO;
using SceneKit;
using Scanner.App.Controls;
using Scanner.Capture.Sessions;
using UIKit;

namespace Scanner.App.iOS.Handlers;

/// <summary>The 3D preview on iOS: SceneKit shows the textured model (or the points), and its own camera control
/// orbits and zooms.</summary>
public sealed class PointCloudViewHandler : ViewHandler<PointCloudView, SCNView>
{
    public static readonly IPropertyMapper<PointCloudView, PointCloudViewHandler> PropertyMapper =
        new PropertyMapper<PointCloudView, PointCloudViewHandler>(ViewMapper)
        {
            [nameof(PointCloudView.Points)] = (handler, _) => handler.ShowScene(),
            [nameof(PointCloudView.Model)] = (handler, _) => handler.ShowScene(),
        };

    public PointCloudViewHandler() : base(PropertyMapper)
    {
    }

    protected override SCNView CreatePlatformView() => new(CoreGraphics.CGRect.Empty, (NSDictionary?)null)
    {
        AllowsCameraControl = true,
        AutoenablesDefaultLighting = true,
        BackgroundColor = UIColor.FromRGB(0.12f, 0.12f, 0.14f),
    };

    private void ShowScene()
    {
        if (PlatformView is not { } view) return;
        var content = VirtualView.Model is { } model ? LoadModel(model)
            : VirtualView.Points is { Length: > 0 } points ? PointsNode(points)
            : null;
        var scene = SCNScene.Create();
        if (content is not null)
        {
            // Centre the piece at the origin and put a camera in front of it, a little above, looking at it.
            SCNVector3 min = default, max = default;
            content.GetBoundingBox(ref min, ref max);
            var centre = new SCNVector3((min.X + max.X) / 2, (min.Y + max.Y) / 2, (min.Z + max.Z) / 2);
            float size = MathF.Max(0.02f, MathF.Max(max.X - min.X, MathF.Max(max.Y - min.Y, max.Z - min.Z)));
            content.Position = new SCNVector3(-centre.X, -centre.Y, -centre.Z);
            scene.RootNode.AddChildNode(content);
            var camera = new SCNNode
            {
                Camera = new SCNCamera { ZNear = size / 100, ZFar = size * 100 },
                Position = new SCNVector3(0, size * 0.8f, size * 1.8f),
            };
            camera.Look(new SCNVector3(0, 0, 0));
            scene.RootNode.AddChildNode(camera);
            view.PointOfView = camera;
        }
        view.Scene = scene;
    }

    private static SCNNode LoadModel(TexturedScene model)
    {
        // ModelIO reads the OBJ and its MTL, whose textures are the session's photos next to it.
        using var url = NSUrl.FromFilename(Path.Combine(model.Directory, TexturedModelFile.Model));
        var asset = new MDLAsset(url);
        asset.LoadTextures();
        var node = new SCNNode();
        foreach (var child in SCNScene.FromAsset(asset).RootNode.ChildNodes) node.AddChildNode(child);
        return node;
    }

    private static SCNNode PointsNode(Vector3[] points)
    {
        var vertices = points.Select(p => new SCNVector3(p.X, p.Y, p.Z)).ToArray();
        var indices = Enumerable.Range(0, points.Length).ToArray();
        var bytes = new byte[indices.Length * sizeof(int)];
        Buffer.BlockCopy(indices, 0, bytes, 0, bytes.Length);
        using var data = NSData.FromArray(bytes);
        var element = SCNGeometryElement.FromData(data, SCNGeometryPrimitiveType.Point, points.Length, sizeof(int));
        element.PointSize = 3;
        element.MinimumPointScreenSpaceRadius = 1.5f;
        element.MaximumPointScreenSpaceRadius = 4;
        var geometry = SCNGeometry.Create([SCNGeometrySource.FromVertices(vertices)], [element]);
        geometry.FirstMaterial = new SCNMaterial { LightingModelName = SCNLightingModel.Constant };
        geometry.FirstMaterial.Diffuse.Contents = UIColor.FromRGB(0.4f, 0.85f, 1f);
        return SCNNode.FromGeometry(geometry);
    }
}
