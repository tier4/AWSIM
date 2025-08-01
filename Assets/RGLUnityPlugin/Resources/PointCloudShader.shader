// Based on: http://answers.unity.com/answers/1437718/view.html
Shader "Custom/PointCloudShader" {
    Properties {
    _DefaultColor ("Default Color", Color) = (1, 1, 1, 1)
    _PointSize ("Point Size", float) = 0.1
    _PointShape ("Point Shape: 0-flatsquare(default), 1-box, 2-pyramid", int) = 0
    _MinColoringHeight ("Min Coloring Height", Float) = 0
    _MaxColoringHeight ("Max Coloring Height", Float) = 10
    }
    SubShader {
    LOD 200
    Tags { "RenderType"="Opaque" "ForceNoShadowCasting"="True" }
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
            float4 pos : POSITION;
            float4 color : COLOR0;
        };
        struct geomOut {
            float4 pos : POSITION;
            float4 color : COLOR0;
        };

        static const float3 _FlatSquareVertices[4] = {
            float3(-1, -1, 0),
            float3(-1,  1, 0),
            float3( 1, -1, 0),
            float3( 1,  1, 0)
        };

        static const int3 _FlatSquareIndices[2] = {
            int3(0, 1, 3),
            int3(0, 3, 2),
        };

        static const float3 _BoxVertices[8] = {
            float3(-1, -1, -1),
            float3( 1, -1, -1),
            float3( 1,  1, -1),
            float3(-1,  1, -1),
            float3(-1,  1,  1),
            float3( 1,  1,  1),
            float3( 1, -1,  1),
            float3(-1, -1,  1)
        };

        static const int3 _BoxIndices[12] = {
            int3(1, 2, 0),
            int3(2, 3, 0),
            int3(4, 3, 2),
            int3(5, 4, 2),
            int3(5, 2, 1),
            int3(6, 5, 1),
            int3(4, 7, 0),
            int3(3, 4, 0),
            int3(7, 4, 5),
            int3(6, 7, 5),
            int3(7, 6, 0),
            int3(6, 1, 0)
        };

        static const float3 _PyramidVertices[5] = {
            float3( 0,  1,  0),
            float3(-1, -1,  1),
            float3( 1, -1,  1),
            float3( 1, -1, -1),
            float3(-1, -1, -1)
        };

        static const int3 _PyramidIndices[6] = {
            int3(2, 1, 0),
            int3(3, 2, 0),
            int3(4, 3, 0),
            int3(1, 4, 0),
            int3(4, 1, 2),
            int3(4, 2, 3)
        };

        float _MinColoringHeight;
        float _MaxColoringHeight;
        half4 _DefaultColor;
        half4 _Colors[6];
        int _ColorsNum;
        vertexOut vert (vertexIn i) {
            if (_ColorsNum < 1) {
                _Colors[0] = _DefaultColor;
                _ColorsNum = 1;
            }

            if (_ColorsNum == 1 || _MinColoringHeight >= _MaxColoringHeight) {
                vertexOut OUT;
                OUT.pos = i.pos;
                OUT.color = _Colors[0];
                return OUT;
            }

            float pos_z = i.pos[1];
            float range = abs(_MaxColoringHeight - _MinColoringHeight);
            float rangeStep = range / (_ColorsNum - 1);

            float4 outColor;
            if (pos_z < _MinColoringHeight || rangeStep <= 0) {
                outColor = _Colors[0];
            } else if (pos_z > _MaxColoringHeight) {
                outColor = _Colors[_ColorsNum - 1];
            } else {
                for (int i = 0; i < _ColorsNum - 1; i++) {
                    float currentMin = _MinColoringHeight + rangeStep * i;
                    float currentMax = _MinColoringHeight + rangeStep * (i + 1);
                    if(pos_z <= currentMax) {
                        float weight = (pos_z - currentMin) / (currentMax - currentMin);
                        outColor = lerp(_Colors[i], _Colors[i + 1], weight);
                        break;
                    }
                }
            }
            vertexOut OUT;
            OUT.pos = i.pos;
            OUT.color = outColor;
            return OUT;
        }

        float _PointSize;
        int _PointShape;
        [maxvertexcount(36)]
        void geom(point vertexOut IN[1], inout TriangleStream<geomOut> OutputStream)
        {
            geomOut OUT;
            OUT.color = IN[0].color;

            if (_PointShape == 1) {  // Box
                for (int i = 0; i < 12; i++) {
                    for (int j = 0; j < 3; j++) {
                        OUT.pos = IN[0].pos + mul(float4(_BoxVertices[_BoxIndices[i][j]], 0) * _PointSize / 2., UNITY_MATRIX_V);
                        OUT.pos = UnityObjectToClipPos(OUT.pos.xyz);
                        OutputStream.Append(OUT);
                    }
                    OutputStream.RestartStrip();
                }
            } else if (_PointShape == 2) {  // Pyramid
                for (int i = 0; i < 6; i++) {
                    for (int j = 0; j < 3; j++) {
                        OUT.pos = IN[0].pos + mul(float4(_PyramidVertices[_PyramidIndices[i][j]], 0) * _PointSize / 2., UNITY_MATRIX_V);
                        OUT.pos = UnityObjectToClipPos(OUT.pos.xyz);
                        OutputStream.Append(OUT);
                    }
                    OutputStream.RestartStrip();
                }
            } else {  // FlatSquare
                for (int i = 0; i < 2; i++) {
                    for (int j = 0; j < 3; j++) {
                        OUT.pos = IN[0].pos + mul(float4(_FlatSquareVertices[_FlatSquareIndices[i][j]], 0) * _PointSize / 2., UNITY_MATRIX_V);
                        OUT.pos = UnityObjectToClipPos(OUT.pos.xyz);
                        OutputStream.Append(OUT);
                    }
                    OutputStream.RestartStrip();
                }
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