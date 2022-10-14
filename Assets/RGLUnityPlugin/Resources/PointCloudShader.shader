// Based on: http://answers.unity.com/answers/1437718/view.html
Shader "Custom/PointCloudShader" {
    Properties {
    _Radius ("Sphere Radius", float) = 1.0
    _FirstColor ("Color in First", Color) = (1, 0, 0, 1)
    _SecondColor("Color in Second", Color) = (1, 1, 0, 1)
    _ThirdColor("Color in Second", Color) = (1, 1, 0, 1)

    _FirstDistance ("First Distance", Float) = 0.5
    _SecondDistance ("Second Distance", Float) = 1
    _ThirdDistance ("Third Distance", Float) = 1.5
    }
    SubShader {
    LOD 200
    Tags { "RenderType"="Opaque" }
    Pass {
        CGPROGRAM
        #pragma vertex vert
        #pragma fragment frag
        #pragma geometry geom
        #pragma target 4.0
        #include "UnityCG.cginc"
        struct vertexIn {
            float4 pos : POSITION;
            float4 color : COLOR;
        };
        struct vertexOut {
            float4 pos : SV_POSITION;
            float4 color : COLOR0;
            float3 normal : NORMAL;
            float r : TEXCOORD0;
        };
        struct geomOut {
            float4 pos : POSITION;
            float4 color : COLO0R;
            float3 normal : NORMAL;
        };

        float4 _FirstColor; 
        float4 _SecondColor; 
        float4 _ThirdColor;
        half4 _resultColor;
        float _FirstDistance;
        float _SecondDistance;
        float _ThirdDistance;

        vertexOut vert (vertexIn i) {
            vertexOut o;
            o.pos = UnityObjectToClipPos(i.pos);
            float pos_z = i.pos[1];

            if(pos_z < _FirstDistance) {
                _resultColor = _FirstColor;
            } else if (pos_z < _SecondDistance) {
                half weight = (pos_z - _FirstDistance) / (_SecondDistance - _FirstDistance);
                _resultColor = lerp(_FirstColor, _SecondColor, weight);
            } else if (pos_z < _ThirdDistance) {
                half weight = (pos_z - _SecondDistance) / (_ThirdDistance - _SecondDistance);
                _resultColor = lerp(_SecondColor, _ThirdColor, weight);
            } else {
                _resultColor = _ThirdColor;
            }
            o.color = _resultColor;
            o.normal = ObjSpaceViewDir(o.pos);
            o.r = i.pos;
            return o;
        }
 
        float _Radius; 
        [maxvertexcount(6)]
        void geom(point vertexOut IN[1], inout TriangleStream<geomOut> OutputStream)
        {
            float2 p[3];
            p[0] = float2(-_Radius, -_Radius);
            p[1] = float2(-_Radius, _Radius);
            p[2] = float2(_Radius, _Radius);;
    
            geomOut OUT;
            OUT.color = IN[0].color;
            OUT.normal = IN[0].normal;
    
            for (int i = 0; i < 3; i++) {
                p[i].x *= _ScreenParams.y / _ScreenParams.x;
                OUT.pos = IN[0].pos + float4(p[i],0,0) / 2.;
                OutputStream.Append(OUT);
            }

            float2 p2[3];
            p2[0] = float2(-_Radius, -_Radius);
            p2[1] = float2(_Radius, _Radius);
            p2[2] = float2(_Radius, -_Radius);
    
            for (int i = 0; i < 3; i++) {
                p2[i].x *= _ScreenParams.y / _ScreenParams.x;
                OUT.pos = IN[0].pos + float4(p2[i],0,0) / 2.;
                OutputStream.Append(OUT);
            }
        }
        float4 frag(geomOut i) : COLOR
        {
            return i.color;
        }
        ENDCG
    }
}

FallBack "Diffuse"
}