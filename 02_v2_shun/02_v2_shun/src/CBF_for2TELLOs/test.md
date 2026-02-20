src2のプログラム内にCBFを組み込みたい．具体的には，以下の制約式を組み込む．

注意点として，実際の実装は以下の座標系仕様のとおりであることに注意する必要がある．
#### 座標系仕様
- **X軸**: 前後方向（+X=前進、-X=後退）
- **Y軸**: 上下方向（+Y=上昇、-Y=下降）
- **Z軸**: 左右方向（+Z=右、-Z=左）
ただし，このファイル内ではX軸が左右方向，Y軸が前後方向，Z軸が上下方向としている．


まず，パラメータを次のように定義する．ただし，いずれのパラメータは仮ぎめである．
K_1=0.009
K_2=0.009
\alpha_1=1.0
\alpha_2=1.0
\alpha_3=1.0
\alpha_4=1.0
\alpha_5=1.0
\Delta=0

v_{ref}=0.0005

また，各ドローンの位置は(x,y)，速度は(\dot{x},\dot{y})，制御入力は(u_x,u_y)を用いて表すことにする．さらに，障害物の位置は(x_o,y_o)を用いて表すことにする．
(x_o,y_o)=(-0.5,-0.2)

ダイナミクスが以下のとき，
\dfrac{d}{dt}\begin{bmatrix}
x\\
y\\
\dot{x}\\
\dot{y}
\end{bmatrix}=\begin{bmatrix}
0&0&1&0\\
0&0&0&1\\
0&0&0&0\\
0&0&0&0\\
\end{bmatrix}\begin{bmatrix}
x\\
y\\
\dot{x}\\
\dot{y}
\end{bmatrix}+\begin{bmatrix}
0&0\\
0&0\\
K_1&0\\
0&K_2
\end{bmatrix}\begin{bmatrix}
u_x\\
u_y
\end{bmatrix}


与えたい制約式は以下の3式
2\dot{x}^2+2(\alpha_2+\alpha_3)(x-x_o)\dot{x}+2\dot{y}^2+2(\alpha_2+\alpha_3)(y-y_o)\dot{y}+2K_1(x-x_o)u_x+2K_2(y-y_o)u_y+\alpha_1\alpha_2(x-x_o)^2+\alpha_1\alpha_2(y-y_o)^2-\alpha_1\alpha_2\Delta^2\geq0

2(¥dot{x}_i-¥dot{x}_j)u_x+alpha_4(¥dot{x}_i-¥dot{x}_j)^2-alpha_4v_{ref}^2¥geq0

2(¥dot{y}_i-¥dot{y}_j)u_y+alpha_5(¥dot{y}_i-¥dot{y}_j)^2-alpha5v_{ref}^2¥geq0