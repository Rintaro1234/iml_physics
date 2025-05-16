/*!
@file main.cpp

@brief 球体の放物線運動(GLFW版)

@author Makoto Fujisawa
@date   2022-03
*/


#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "glfw3dll.lib")

#define GL_SILENCE_DEPRECATION	// mac環境でgluを使っている場合の非推奨warningの抑制


//-----------------------------------------------------------------------------
// インクルードファイル
//-----------------------------------------------------------------------------
#include "utils.h"

// ImGUI
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

// Bullet
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include <btBulletDynamicsCommon.h>

using namespace std;


//-----------------------------------------------------------------------------
// 定数・グローバル変数
//-----------------------------------------------------------------------------

// BULLET
btDynamicsWorld* g_dynamicsworld = 0;	//!< Bulletワールド
btRigidBody* g_PlaneTerrain = 0; //!< 地面
btRigidBody* g_ballbody = 0;		//!< 球体

// 光源位置/色 (shadow mapなどで使う場合用にグローバル変数にしている)
const glm::vec4 RX_LIGHT0_POS(0.0f, 0.0f, 1.0f, 1.0f);
const glm::vec4 RX_LIGHT1_POS(-1.0f, -10.0f, -1.0f, 0.0f);
const glm::vec4 RX_LIGHT_AMBI(0.3f, 0.3f, 0.3f, 1.0f);
const glm::vec4 RX_LIGHT_DIFF(1.0f, 1.0f, 1.0f, 1.0f);
const glm::vec4 RX_LIGHT_SPEC(1.0f, 1.0f, 1.0f, 1.0f);

// ウィンドウ/アニメーション/描画関連変数
int g_winw = 1024;							//!< 描画ウィンドウの幅
int g_winh = 1024;							//!< 描画ウィンドウの高さ
bool g_animation_on = false;				//!< アニメーションON/OFF
int g_currentstep = 0;						//!< 現在のステップ数

// 物理シミュレーション関連定数/変数
const GLfloat RX_GROUND = -0.5f;			//!< 地面の位置(y座標)
const int MAX_TRAJ = 1024;					//!< ボール中心座標を格納する配列の最大サイズ
const btVector3 RX_INIT_POS(-1, 0.5, 0);	//!< ボールの初期位置
const btQuaternion RX_INIT_QROT(1, 0, 0, 1);//!< ボールの初期回転

float g_dt = 0.01;							//!< 時間ステップ幅Δt

// 形の定義
float g_ballrad = 0.1;						//!< 半径		

btVector3 g_frc = btVector3(3, 0, 0);		//!< 初期外力

float g_ball_restitution = 0.8;					//!< 反発係数(ボール)

btVector3 g_trajectories[MAX_TRAJ];			//!< ボールの軌跡を格納する配列
int g_num_trajectory = 0;					//!< 格納されたボールの座標の数

// 地面
float g_plane_restitution = 0.7; //!< 反発係数（地面）



//-----------------------------------------------------------------------------
// アプリケーション制御関数
//-----------------------------------------------------------------------------
/*!
* アニメーションON/OFF
* @param[in] on trueでON, falseでOFF
*/
bool switchanimation(int on)
{
	g_animation_on = (on == -1) ? !g_animation_on : (on ? true : false);
	return g_animation_on;
}
/*!
* 現在の画面描画を画像ファイルとして保存(連番)
* @param[in] stp 現在のステップ数(ファイル名として使用)
*/
void savedisplay(const int& stp)
{
	static int nsave = 1;
	string fn = CreateFileName("img_", ".bmp", (stp == -1 ? nsave++ : stp), 5);
	saveFrameBuffer(fn, g_winw, g_winh);
	std::cout << "saved the screen image to " << fn << std::endl;
}

/*!
* 初期化関数
*  - プログラム起動時に一回だけだけ実行したい処理はここに書く
*/
void InitBullet(void)
{
	// 衝突検出の選択
	btDefaultCollisionConfiguration* config = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(config);

	// ブロードフェーズ法の設定
	btDbvtBroadphase* broadphase = new btDbvtBroadphase();

	// 拘束（剛体間リンク）のソルバ設定
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	// Bulletのワールド作成
	g_dynamicsworld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
	g_dynamicsworld->setGravity(btVector3(0, -9.8, 0));

	// 地面の生成
	// 地面の形状を設定
	btStaticPlaneShape* terrain_shape = new btStaticPlaneShape(btVector3(0, 1, 0), RX_GROUND);
	// 地面の初期位置・姿勢
	btDefaultMotionState* terrain_motion_state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0), btVector3(0, 0, 0)));
	// 地面の剛体の作成
	g_PlaneTerrain = new btRigidBody(0, terrain_motion_state, terrain_shape, btVector3(0, 0, 0));
	// 反発係数
	g_PlaneTerrain->setRestitution(g_plane_restitution);
	// ワールドに地面の剛体を追加
	g_dynamicsworld->addRigidBody(g_PlaneTerrain);

	// 球体形状を設定
	btCollisionShape* sphere_shape = new btCylinderShapeZ(btVector3(g_ballrad, g_ballrad, g_ballrad));

	// 剛体の宣言
	btVector3 pos = RX_INIT_POS; //!< 中心座標
	btQuaternion qrot = RX_INIT_QROT; //!< 回転
	btScalar mass = 0.03; //!< 質量

	// 球体の初期位置・姿勢
	btDefaultMotionState* motion_state = new btDefaultMotionState(btTransform(qrot, pos));

	//慣性モーメントの計算
	btVector3 inertia(0, 0, 0);
	sphere_shape->calculateLocalInertia(mass, inertia);

	// 剛体オブジェクトの生成(質量、位置姿勢、形状、　慣性モーメント)
	g_ballbody = new btRigidBody(mass, motion_state, sphere_shape, inertia);

	// 反発係数
	g_ballbody->setRestitution(g_ball_restitution);

	// ワールドに剛体オブジェクトを追加
	g_dynamicsworld->addRigidBody(g_ballbody);
}

void Init(void)
{
	// OpenGLのバージョンチェック
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	cout << "Vendor: " << glGetString(GL_VENDOR) << endl;
	cout << "Renderer: " << glGetString(GL_RENDERER) << endl;

	// GLEWの初期化
	GLenum err = glewInit();
	if (err != GLEW_OK) cout << "GLEW Error : " << glewGetErrorString(err) << endl;

	// 描画系フラグ設定(アンチエイリアス,デプステスト,隠面除去,法線計算,点描画)
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_POINT_SMOOTH);

	// 光源&材質描画設定
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, glm::value_ptr(RX_LIGHT0_POS));
	glLightfv(GL_LIGHT0, GL_DIFFUSE, glm::value_ptr(RX_LIGHT_DIFF));
	glLightfv(GL_LIGHT0, GL_SPECULAR, glm::value_ptr(RX_LIGHT_SPEC));
	glLightfv(GL_LIGHT0, GL_AMBIENT, glm::value_ptr(RX_LIGHT_AMBI));

	glShadeModel(GL_SMOOTH);

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	// BULLETの初期化
	InitBullet();
}

void CleanBullet(void) {
	// オブジェクトの破棄
	delete g_ballbody->getMotionState();
	g_dynamicsworld->removeRigidBody(g_ballbody);
	delete g_ballbody;
	delete g_PlaneTerrain->getMotionState();
	g_dynamicsworld->removeRigidBody(g_PlaneTerrain);
	delete g_PlaneTerrain;

	// ワールド破棄
	delete g_dynamicsworld->getBroadphase();
	delete g_dynamicsworld;
}

/*!
* シミュレーションのリセット
*  - 自分でグローバル変数を追加した場合はその値のリセット処理の追加を忘れずに
*/
void reset(void)
{	
	CleanBullet();
	InitBullet();
	g_currentstep = 0;
	switchanimation(0);
}

//-----------------------------------------------------------------------------
// OpenGL/GLFWコールバック関数
//-----------------------------------------------------------------------------
/*!
* 再描画イベントコールバック関数
*/
void Display(void)
{
	// ビューポート,透視変換行列,モデルビュー変換行列の設定
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float fovy = 45.0f;
	float aspect = (float)g_winw / g_winh;
	float znear = 0.01f;
	float zfar = 20.0f;
	glm::mat4 mp = glm::perspective(fovy, aspect, znear, zfar);
	glMultMatrixf(glm::value_ptr(mp));
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// 描画バッファのクリア
	glClearColor(0.8, 0.8, 0.9, 1.0);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

		// 地面の描画
		glPushMatrix();
			const btStaticPlaneShape* planeShape = static_cast<const btStaticPlaneShape*>(g_PlaneTerrain->getCollisionShape());
			btVector3 planeNorm = planeShape->getPlaneNormal();
			float planeCons = planeShape->getPlaneConstant();
			float farX = aspect * (zfar * sin(fovy / 2 * 3.141592654 / 180)) + 1.0f;
			float nearX = aspect * (znear * sin(fovy / 2 * 3.141592654 / 180)) + 1.0f;
			float rightFarY = (-zfar * planeNorm[2] - planeNorm[0] * farX + planeCons) / planeNorm[1];
			float leftFarY = (-zfar * planeNorm[2] + planeNorm[0] * farX + planeCons) / planeNorm[1];
			float rightNearY = (-znear * planeNorm[2] - planeNorm[0] * nearX + planeCons) / planeNorm[1];
			float leftNearY = (-znear * planeNorm[2] + planeNorm[0] * nearX + planeCons) / planeNorm[1];

			glEnable(GL_LIGHTING);
			glBegin(GL_POLYGON);
			glVertex3f(farX, rightFarY, -zfar);
			glVertex3f(-farX, leftFarY, -zfar);
			glVertex3f(-nearX, leftNearY, -znear);
			glVertex3f(nearX, rightNearY, -znear);
			glEnd();
		glPopMatrix();

		glTranslatef(0, 0, -5);	// 視点をz方向に移動
		
		// 剛体の描画
		glPushMatrix();
			btScalar m[16];
			btDefaultMotionState* motion = static_cast<btDefaultMotionState*>(g_ballbody->getMotionState());
			motion->m_graphicsWorldTrans.getOpenGLMatrix(m);
#ifdef BT_USE_DOUBLE_PRECISION
			glMultMatrixd(m);
#else
			glMultMatrixf(m);
#endif
			// 投射オブジェクト
			glEnable(GL_LIGHTING);
			glColor3f(0.1, 0.5, 1.0);
			glScalef(2 * g_ballrad,  2 * g_ballrad, g_ballrad);
			DrawCylinderVBO();	// VBOによる球体メッシュ描画
		glPopMatrix();

		// 軌跡
		if (g_num_trajectory) {
			// ボールの中心座標の軌跡の描画
			// GL_LINE_STRIPを使ってみよう
			glDisable(GL_LIGHTING);
		}

	glPopMatrix();
}

/*!
 * タイマーイベント処理関数(ある時間間隔で実行)
 */
void Timer(void)
{
	// 球体の放物線運動を計算
	if (g_animation_on) {
		if (g_currentstep == 0) {
			g_ballbody->applyCentralImpulse(g_frc * g_dt);
		}

		// 球の軌跡の格納
		//  配列g_trajectoriesに現在の位置座標をスタックしていく．
		//  g_num_trajectoryをすでに格納された数とすると...
		//  (g_num_trajectory >= MAX_TRAJとなったときのエラー処理も忘れずに)


		// bulletのステップを進める
		if (g_dynamicsworld) {
			g_dynamicsworld->stepSimulation(g_dt, 1);
		}

		g_currentstep++;
	}
}


/*!
* キーボードイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] key キーの種類 -> https://www.glfw.org/docs/latest/group__keys.html
* @param[in] scancode キーのスキャンコード(プラットフォーム依存)
* @param[in] action アクション(GLFW_PRESS:キーを押す, GLFW_RELESE:キーを離す，GLFW_REPEAT:キーリピート機能時)
* @param[in] mods 修飾キー(CTRL,SHIFT,ALT) -> https://www.glfw.org/docs/latest/group__mods.html
*/
void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (ImGui::GetIO().WantCaptureKeyboard) return;	// ImGUIウィンドウ上でのキーボードイベント時
	if (action == GLFW_PRESS) {
		switch (key) {
		case GLFW_KEY_ESCAPE:	// ESC,Qキーでアプリケーション終了
		case GLFW_KEY_Q:
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;

		case GLFW_KEY_S: // SキーでアニメーションON/OFF
			switchanimation(-1);
			break;
		case GLFW_KEY_SPACE: // スペースキーでアニメーションを1ステップだけ進める
			g_animation_on = true; Timer(); g_animation_on = false;
			break;

		case GLFW_KEY_R: // Rキーでシーンリセット
			reset();
			break;

		default:
			break;
		}
	}
}


/*!
* リサイズイベント処理関数
* @param[in] window コールバック関数を呼んだウィンドウハンドル
* @param[in] w キャンバス幅(ピクセル数)
* @param[in] h キャンバス高さ(ピクセル数)
*/
void Resize(GLFWwindow* window, int w, int h)
{
	g_winw = w; g_winh = h;
	glViewport(0, 0, g_winw, g_winh);
}

/*!
* ImGUIのウィジット配置
*  - ImGUI/imgui_demo.cppを参考に ( https://github.com/ocornut/imgui#demo )
* @param[in] window コールバック関数を呼んだウィンドウハンドル
*/
void SetImGUI(GLFWwindow* window)
{
	ImGui::Text("simulation:");
	if (ImGui::Button("start/stop")) { switchanimation(-1); } ImGui::SameLine();
	if (ImGui::Button("run a step")) { g_animation_on = true; Timer(); g_animation_on = false; }
	if (ImGui::Button("reset")) { reset(); }
	ImGui::Separator();
#ifdef BT_USE_DOUBLE_PRECISION
	ImGui::InputDouble("force", &(g_frc[0]), 0.1f, 1.0f, "%.1f");
#else
	ImGui::InputFloat("force", &(g_frc[0]), 0.1f, 1.0f, "%.1f");
#endif
	ImGui::InputFloat("dt", &(g_dt), 0.001f, 0.01f, "%.3f");
	ImGui::Separator();
	if (ImGui::Button("save screenshot")) { savedisplay(-1); }
	if (ImGui::Button("quit")) { glfwSetWindowShouldClose(window, GL_TRUE); }
}

void Clean()
{
}




/*!
* メインルーチン
* @param[in] argc コマンドライン引数の数
* @param[in] argv コマンドライン引数
*/
int main(int argc, char* argv[])
{
	if (!glfwInit()) return 1;
	glfwSetErrorCallback(glfw_error_callback);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#ifdef __APPLE__
	glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
#endif

	// Create window
	GLFWwindow* window = glfwCreateWindow(g_winw, g_winh, "OpenGL Application", NULL, NULL);
	if (window == NULL) return 1;

	// Set glfw window as current OpenGL rendering context
	glfwMakeContextCurrent(window);
	glewExperimental = GL_TRUE;
	glfwSwapInterval(0); // Disable vsync

	// Initilization
	Init();

	// Setup callback functions
	glfwSetKeyCallback(window, Keyboard);
	glfwSetFramebufferSizeCallback(window, Resize);
	Resize(window, g_winw, g_winh);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL2_Init();

	// Settings for timer to display FPS on ImGUI window
	float cur_time = 0.0f, last_time = 0.0f, elapsed_time = 0.0f;
	glfwSetTime(0.0);	// Initialize the glfw timer

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		// Poll and handle events (inputs, window resize, etc.)
		glfwPollEvents();

		// OpenGL Rendering & Animation function
		Display();

		// Timer
		cur_time = glfwGetTime();
		elapsed_time = cur_time - last_time;
		if (elapsed_time >= g_dt) {
			Timer();
			last_time = glfwGetTime();
		}

		// Start the ImGui frame
		ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// GUI
		ImGui::Begin("ImGui Window");
		ImGui::Text("Framerate: %.3f ms/frame (%.1f fps)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Separator();
		SetImGUI(window);
		ImGui::End();

		// Rendering of the ImGUI frame in opengl canvas
		ImGui::Render();
		ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
	}

	// Cleanup
	Clean();
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();


	return 0;
}