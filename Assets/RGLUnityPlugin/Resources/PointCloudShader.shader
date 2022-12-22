// Based on: http://answers.unity.com/answers/1437718/view.html
Shader "Custom/PointCloudShader" {
    Properties {
    _Color ("Flat Color", Color) = (1, 1, 1, 1)
    _PointSize ("Point Size", float) = 0.1
    _PointShape ("Point Shape: 0-flatsquare, 1-box, 2-pyramid(default)", int) = 1
    _MinColoringHeight ("Min Coloring Height", Float) = 0
    _MaxColoringHeight ("Max Coloring Height", Float) = 10
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
            float4 pos : POSITION;
            float4 color : COLOR0;
        };
        struct geomOut {
            float4 pos : POSITION;
            float4 color : COLO0R;
            float3 normal : NORMAL;
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

        static const float3 _FlatSquareVertices[4] = {
            float3(-1, 0, -1),
	        float3(-1, 0,  1),
            float3( 1, 0, -1),
	        float3( 1, 0,  1)
        };

        static const int3 _FlatSquareIndices[4] = {
            int3(0, 1, 3),
            int3(0, 3, 2),

            int3(3, 1, 0),
            int3(2, 3, 0)
        };

        static const int3 _BoxIndices[12] = {
            int3(0, 2, 1),
            int3(0, 3, 2),
            int3(2, 3, 4),
            int3(2, 4, 5),
            int3(1, 2, 5),
            int3(1, 5, 6),
            int3(0, 7, 4),
            int3(0, 4, 3),
            int3(5, 4, 7),
            int3(5, 7, 6),
            int3(0, 6, 7),
            int3(0, 1, 6)
        };

        static const float3 _PyramidVertices[5] = {
            float3( 0,  1,  0),
	        float3(-1, -1,  1),
            float3( 1, -1,  1),
            float3( 1, -1, -1),
	        float3(-1, -1, -1)
        };

        static const int3 _PyramidIndices[6] = {
            int3(0, 1, 2),
            int3(0, 2, 3),
            int3(0, 3, 4),
            int3(0, 4, 1),

            int3(2, 1, 4),
            int3(3, 2, 4)
        };

        half4 _resultColor;
        float _MinColoringHeight;
        float _MaxColoringHeight;
        half4 _Color;
        half4 _Colors[6];
        int _ColorsNum;
        vertexOut vert (vertexIn i) {
            vertexOut o;
            o.pos = i.pos;

            float pos_z = i.pos[1];

            if (_ColorsNum < 1) {
                _Colors[0] = _Color;
            }

            float range = abs(_MaxColoringHeight - _MinColoringHeight);
            float rangeStep = range / _ColorsNum;

            bool isColorSet = false;
            if(pos_z < _MaxColoringHeight || _ColorsNum == 1) {
                _resultColor = _Colors[0];
                isColorSet = true;
            }

            if (rangeStep > 0) {
                for (int i = 1; i < _ColorsNum; i++) {
                    half currentMin = _MinColoringHeight + rangeStep * i;
                    half currentMax = _MinColoringHeight + rangeStep * (i + 1);
                    if(pos_z < currentMax) {
                            half weight = (pos_z - currentMin) / (currentMax - currentMin);
                            _resultColor = lerp(_Colors[i - 1], _Colors[i], weight);
                        isColorSet = true;
                        break;
                    }
                }
            }

            if (!isColorSet) {
                _resultColor = _Colors[_ColorsNum - 1];
            }

            o.color = _resultColor;
            return o;
        }
 
        float _PointSize;
        int _PointShape;
        [maxvertexcount(36)]
        void geom(point vertexOut IN[1], inout TriangleStream<geomOut> OutputStream)
        {
            geomOut OUT;
            OUT.color = IN[0].color;
            OUT.normal = ObjSpaceViewDir(UnityObjectToClipPos(IN[0].pos));

            if (_PointShape == 0) {  // FlatSquare
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 3; j++) {
                        OUT.pos = IN[0].pos + float4(_FlatSquareVertices[_FlatSquareIndices[i][j]], 0) * _PointSize / 2.;
                        OUT.pos = UnityObjectToClipPos(OUT.pos.xyz);
                        OutputStream.Append(OUT);
                    }
                    OutputStream.RestartStrip();
                }
            } else if (_PointShape == 1) {  // Box
                for (int i = 0; i < 12; i++) {
                    for (int j = 0; j < 3; j++) {
                        OUT.pos = IN[0].pos + float4(_BoxVertices[_BoxIndices[i][j]], 0) * _PointSize / 2.;
                        OUT.pos = UnityObjectToClipPos(OUT.pos.xyz);
                        OutputStream.Append(OUT);
                    }
                    OutputStream.RestartStrip();
                }
            } else {  // Pyramid
                for (int i = 0; i < 6; i++) {
                    for (int j = 0; j < 3; j++) {
                        OUT.pos = IN[0].pos + float4(_PyramidVertices[_PyramidIndices[i][j]], 0) * _PointSize / 2.;
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