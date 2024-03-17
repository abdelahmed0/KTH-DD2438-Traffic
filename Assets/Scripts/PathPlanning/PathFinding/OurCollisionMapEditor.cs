using System;
using UnityEditor;
using UnityEngine;


[CustomEditor(typeof(OurCollisionMap))]
public class OurCollisionMapEditor : Editor
{
    public override void OnInspectorGUI()
    {
        var changed = DrawDefaultInspector();

        OurCollisionMap myScript = (OurCollisionMap)target;
        if (changed)
        {

        }


        if (GUILayout.Button("Reload"))
        {
            myScript.ResetMap();
        }
    }
}