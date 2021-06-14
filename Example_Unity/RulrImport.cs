using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

public class RulrImport {
	class CameraLoader
	{
		public class OpenGLLoader
		{
			public float[] projectionMatrix;
			public float[] viewMatrix;
		};

		public OpenGLLoader OpenGL = new OpenGLLoader();
	};

	/// <summary>
	/// Extract translation from transform matrix.
	/// </summary>
	/// <param name="matrix">Transform matrix. This parameter is passed by reference
	/// to improve performance; no changes will be made to it.</param>
	/// <returns>
	/// Translation offset.
	/// </returns>
	public static Vector3 ExtractTranslationFromMatrix(ref Matrix4x4 matrix)
	{
		Vector3 translate;
		translate.x = matrix.m03;
		translate.y = matrix.m13;
		translate.z = matrix.m23;
		return translate;
	}

	/// <summary>
	/// Extract rotation quaternion from transform matrix.
	/// </summary>
	/// <param name="matrix">Transform matrix. This parameter is passed by reference
	/// to improve performance; no changes will be made to it.</param>
	/// <returns>
	/// Quaternion representation of rotation transform.
	/// </returns>
	public static Quaternion ExtractRotationFromMatrix(ref Matrix4x4 matrix)
	{
		Vector3 forward;
		forward.x = matrix.m02;
		forward.y = matrix.m12;
		forward.z = matrix.m22;

		Vector3 upwards;
		upwards.x = matrix.m01;
		upwards.y = matrix.m11;
		upwards.z = matrix.m21;

		return Quaternion.LookRotation(forward, upwards);
	}

	/// <summary>
	/// Extract scale from transform matrix.
	/// </summary>
	/// <param name="matrix">Transform matrix. This parameter is passed by reference
	/// to improve performance; no changes will be made to it.</param>
	/// <returns>
	/// Scale vector.
	/// </returns>
	public static Vector3 ExtractScaleFromMatrix(ref Matrix4x4 matrix)
	{
		Vector3 scale;
		scale.x = new Vector4(matrix.m00, matrix.m10, matrix.m20, matrix.m30).magnitude;
		scale.y = new Vector4(matrix.m01, matrix.m11, matrix.m21, matrix.m31).magnitude;
		scale.z = new Vector4(matrix.m02, matrix.m12, matrix.m22, matrix.m32).magnitude;
		return scale;
	}

	/// <summary>
	/// Extract position, rotation and scale from TRS matrix.
	/// </summary>
	/// <param name="matrix">Transform matrix. This parameter is passed by reference
	/// to improve performance; no changes will be made to it.</param>
	/// <param name="localPosition">Output position.</param>
	/// <param name="localRotation">Output rotation.</param>
	/// <param name="localScale">Output scale.</param>
	public static void DecomposeMatrix(ref Matrix4x4 matrix, out Vector3 localPosition, out Quaternion localRotation, out Vector3 localScale)
	{
		localPosition = ExtractTranslationFromMatrix(ref matrix);
		localRotation = ExtractRotationFromMatrix(ref matrix);
		localScale = ExtractScaleFromMatrix(ref matrix);
	}

	/// <summary>
	/// Set transform component from TRS matrix.
	/// </summary>
	/// <param name="transform">Transform component.</param>
	/// <param name="matrix">Transform matrix. This parameter is passed by reference
	/// to improve performance; no changes will be made to it.</param>
	public static void SetTransformFromMatrix(Transform transform, ref Matrix4x4 matrix)
	{
		transform.localPosition = ExtractTranslationFromMatrix(ref matrix);
		transform.localRotation = ExtractRotationFromMatrix(ref matrix);
		transform.localScale = ExtractScaleFromMatrix(ref matrix);
	}


	// EXTRAS!

	/// <summary>
	/// Identity quaternion.
	/// </summary>
	/// <remarks>
	/// <para>It is faster to access this variation than <c>Quaternion.identity</c>.</para>
	/// </remarks>
	public static readonly Quaternion IdentityQuaternion = Quaternion.identity;
	/// <summary>
	/// Identity matrix.
	/// </summary>
	/// <remarks>
	/// <para>It is faster to access this variation than <c>Matrix4x4.identity</c>.</para>
	/// </remarks>
	public static readonly Matrix4x4 IdentityMatrix = Matrix4x4.identity;

	/// <summary>
	/// Get translation matrix.
	/// </summary>
	/// <param name="offset">Translation offset.</param>
	/// <returns>
	/// The translation transform matrix.
	/// </returns>
	public static Matrix4x4 TranslationMatrix(Vector3 offset)
	{
		Matrix4x4 matrix = IdentityMatrix;
		matrix.m03 = offset.x;
		matrix.m13 = offset.y;
		matrix.m23 = offset.z;
		return matrix;
	}

	[MenuItem("CONTEXT/Camera/Load Rulr Camera...")]
	private static void LoadCamera(MenuCommand command)
	{
		var result = EditorUtility.OpenFilePanel("Select camera json file", "", "json");
		if(result.Length != 0)
		{
			var jsonEncoded = File.ReadAllText(result);
			var cameraLoader = JsonConvert.DeserializeObject<CameraLoader>(jsonEncoded);

			var camera = command.context as Camera;

			var projectionMatrix = new Matrix4x4();
			var viewMatrix = new Matrix4x4();
			for (int i=0; i<16; i++)
			{
				projectionMatrix[i] = cameraLoader.OpenGL.projectionMatrix[i];
				viewMatrix[i] = cameraLoader.OpenGL.viewMatrix[i];
			}
			
			projectionMatrix[1, 1] = -projectionMatrix[1, 1];
			projectionMatrix[2, 2] = -projectionMatrix[2, 2];
			projectionMatrix[3, 2] = -projectionMatrix[3, 2];

			//lens offset
			projectionMatrix[0, 2] = -projectionMatrix[0, 2];
			projectionMatrix[1, 2] = -projectionMatrix[1, 2];

			camera.projectionMatrix = projectionMatrix;
			var viewInverse = viewMatrix.inverse;

			//rotation
			viewInverse[0, 1] *= -1;
			viewInverse[0, 2] *= -1;

			viewInverse[1, 0] *= -1;
			viewInverse[1, 1] *= -1;
			viewInverse[1, 2] *= -1;

			viewInverse[2, 1] *= -1;

			//translation
			viewInverse[1, 3] *= -1;

			var rotateAboutX = Matrix4x4.identity;
			rotateAboutX[0, 0] = +1;
			rotateAboutX[1, 1] = -1;
			rotateAboutX[2, 2] = -1;

			var rotateAboutZ = Matrix4x4.identity;
			rotateAboutZ[0, 0] = -1;
			rotateAboutZ[1, 1] = -1;
			rotateAboutZ[2, 2] = +1;
			rotateAboutZ[3, 3] = +1;

			viewInverse = rotateAboutX * viewInverse * rotateAboutZ;
			var transform = camera.gameObject.transform;
			SetTransformFromMatrix(transform, ref viewInverse);
		}
	}
}
