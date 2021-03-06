/*
 * demo_Riemannian_pose_infHorLQR.cpp
 *
 * Infinite horizon LQR for R3 x S3 pose.
 *
 * Created on: Nov 2, 2017
 * Author: Andras Kupcsik
 */

#include <stdio.h>
#include <vector>

#include <armadillo>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw_gl2.h>
#include <gfx_ui.h>
#include <window_utils.h>

using namespace arma;


//-------------------------------------------------------------------------
// Holds the sizes of the window and of the OpenGL front-buffer (they might
// be different, for instance on a 4K screen)
//-------------------------------------------------------------------------
struct window_size_t {
	int win_width;
	int win_height;
	int fb_width;
	int fb_height;

	inline float scale_x() const {
		return (float) fb_width / (float) win_width;
	}

	inline float scale_y() const {
		return (float) fb_height / (float) win_height;
	}
};

//-----------------------------------------------

static void error_callback(int error, const char* description) {
	fprintf(stderr, "Error %d: %s\n", error, description);
}

//-----------------------------------------------

arma::mat QuatMatrix(arma::mat q) {
	arma::mat Q;
	Q = { { q(0),-q(1),-q(2),-q(3)},
		{	q(1), q(0),-q(3), q(2)},
		{	q(2), q(3), q(0),-q(1)},
		{	q(3),-q(2), q(1), q(0)}};

	return Q;
}

//-----------------------------------------------

mat QuatToRotMat(vec4 q) {
	float w = q(0);
	float x = q(1);
	float y = q(2);
	float z = q(3);
	mat RotMat(3, 3);
	RotMat << 1 - 2 * y * y - 2 * z * z << 2 * x * y - 2 * z * w << 2 * x * z + 2 * y * w << endr << 2 * x * y + 2 * z * w << 1 - 2 * x * x - 2 * z * z
			<< 2 * y * z - 2 * x * w << endr << 2 * x * z - 2 * y * w << 2 * y * z + 2 * x * w << 1 - 2 * x * x - 2 * y * y << endr;
	return RotMat;
}

//-----------------------------------------------

arma::mat acoslog(arma::mat x) {
	arma::mat acosx(1, x.size());

	for (int n = 0; n <= x.size() - 1; n++) {
		if (x(0, n) >= 1.0)
			x(0, n) = 1.0;
		if (x(0, n) <= -1.0)
			x(0, n) = -1.0;
		if (x(0, n) < 0) {
			acosx(0, n) = acos(x(0, n)) - M_PI;
		} else
			acosx(0, n) = acos(x(0, n));
	}

	return acosx;
}

//-----------------------------------------------

arma::mat Qexpfct(arma::mat u) {
	arma::mat normv = sqrt(pow(u.row(0), 2) + pow(u.row(1), 2) + pow(u.row(2), 2));
	arma::mat Exp(4, u.n_cols);

	Exp.row(0) = cos(normv);
	Exp.row(1) = u.row(0) % sin(normv) / normv;
	Exp.row(2) = u.row(1) % sin(normv) / normv;
	Exp.row(3) = u.row(2) % sin(normv) / normv;

	return Exp;
}

//-----------------------------------------------

arma::mat Qlogfct(arma::mat x) {
	arma::mat fullone;
	fullone.ones(size(x.row(0)));
	arma::mat scale(1, x.size() / 4);
	scale = acoslog(x.row(0)) / sqrt(fullone - pow(x.row(0), 2));

	if (scale.has_nan()) {
		scale = 1.0;
	}

	arma::mat Log(3, x.size() / 4);
	Log.row(0) = x.row(1) % scale;
	Log.row(1) = x.row(2) % scale;
	Log.row(2) = x.row(3) % scale;

	return Log;
}

//-----------------------------------------------

arma::mat Qexpmap(arma::mat u, arma::vec mu) {
	arma::mat x = QuatMatrix(mu) * Qexpfct(u);
	return x;
}

//-----------------------------------------------

arma::mat Qlogmap(arma::mat x, arma::vec mu) {
	arma::mat pole;
	arma::mat Q(4, 4, fill::ones);

	pole = {1,0,0,0};

	if (norm(mu - trans(pole)) < 1E-6)
		Q = { { 1,0,0,0},
			{	0,1,0,0},
			{	0,0,1,0},
			{	0,0,0,1}};
		else
		Q = QuatMatrix(mu);

	arma::mat u;
	u = Qlogfct(trans(Q) * x);

	return u;
}

//-----------------------------------------------

arma::mat Qtransp(vec g, vec h) {
	mat E;
	E << 0.0 << 0.0 << 0.0 << endr << 1.0 << 0.0 << 0.0 << endr << 0.0 << 1.0 << 0.0 << endr << 0.0 << 0.0 << 1.0;
	colvec tmpVec = zeros(4, 1);
	tmpVec.subvec(0, 2) = Qlogmap(h, g);

	vec vm = QuatMatrix(g) * tmpVec;
	double mn = norm(vm, 2);
	mat Ac;
	if (mn < 1E-10) {
		Ac = eye(3, 3);
	}

	colvec uv = vm / mn;

	mat Rpar = eye(4, 4) - sin(mn) * (g * uv.t()) - (1 - cos(mn)) * (uv * uv.t());

	Ac = E.t() * QuatMatrix(h).t() * Rpar * QuatMatrix(g) * E;

	return Ac;
}

//-----------------------------------------------

arma::mat transp(vec g, vec h) {
	mat Ac = eye(6, 6);
	Ac.submat(3, 3, 5, 5) = Qtransp(g.subvec(3, 6), h.subvec(3, 6));
	return Ac;
}

//-----------------------------------------------

arma::vec logmap(vec x, vec mu) {
	vec u = join_vert(x.subvec(0, 2), Qlogmap(x.subvec(3, 6), mu.subvec(3, 6)));
	return u;
}

//-----------------------------------------------

arma::vec expmap(vec u, vec mu) {
	vec x = join_vert(u.subvec(0, 2), Qexpmap(u.subvec(3, 5), mu.subvec(3, 6)));
	return x;
}

//-----------------------------------------------

arma::vec gaussPDF(mat Data, colvec Mu, mat Sigma) {

	int nbVar = Data.n_rows;
	int nbData = Data.n_cols;
	Data = Data.t() - repmat(Mu.t(), nbData, 1);

	vec prob = sum((Data * inv(Sigma)) % Data, 1);

	prob = exp(-0.5 * prob) / sqrt(pow((2 * datum::pi), nbVar) * det(Sigma) + DBL_MIN);

	return prob;
}

//-----------------------------------------------

arma::mat solveAlgebraicRiccati_eig_discrete(mat A, mat B, mat Qx, mat R) {
	// Ajay Tanwani, 2016

	int n = A.n_rows;
	mat Q = (Qx + Qx.t()) / 2; // mat Q here corresponds to (Q+Q')/2 specified in set_problem
	mat G = B * R.i() * B.t();
	mat Z(2 * n, 2 * n);
	Z(span(0, n - 1), span(0, n - 1)) = A + G * inv(A.t()) * Q;
	Z(span(0, n - 1), span(n, 2 * n - 1)) = -G * inv(A.t());
	Z(span(n, 2 * n - 1), span(0, n - 1)) = -inv(A.t()) * Q;
	Z(span(n, 2 * n - 1), span(n, 2 * n - 1)) = inv(A.t());

	//Using diagonalization
	cx_mat V(2 * n, 2 * n), U(2 * n, n);
	cx_vec dd(2 * n);

	arma::eig_gen(dd, V, Z);

	int i = 0;
	for (int j = 0; j < 2 * n; j++) {
		if (norm(dd(j)) < 1) {
			U.col(i) = V.col(j);
			i++;
		}
	}

	mat X = real(U(span(n, 2 * n - 1), span(0, n - 1)) * U(span(0, n - 1), span(0, n - 1)).i());

	return X;
}

//-----------------------------------------------

const arma::fmat COLORS({
	{ 0.00f, 0.00f, 1.00f },
	{ 0.00f, 0.50f, 0.00f },
	{ 1.00f, 0.00f, 0.00f },
	{ 0.00f, 0.75f, 0.75f },
	{ 0.75f, 0.00f, 0.75f },
	{ 0.75f, 0.75f, 0.00f },
	{ 0.25f, 0.25f, 0.25f },
	{ 0.00f, 0.00f, 1.00f },
	{ 0.00f, 0.50f, 0.00f },
	{ 1.00f, 0.00f, 0.00f },
});

//-----------------------------------------------

int main(int argc, char **argv) {

	// Take 4k screens into account (framebuffer size != window size)
	window_size_t window_size;
	window_size.win_width = 780;
	window_size.win_height = 540;
	window_size.fb_width = -1;	// Will be known later
	window_size.fb_height = -1;

	//Setup GUI
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(1);

	GLFWwindow* window = create_window_at_optimal_size(
		"Riemannian Pose InfHorLQR", window_size.win_width, window_size.win_height
	);

	glfwMakeContextCurrent(window);

	glEnable (GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Setup ImGui binding
	ImGui::CreateContext();
	ImGui_ImplGlfwGL2_Init(window, true);

	// white background
	ImVec4 clear_color = ImColor(255, 255, 255);
	ImVector<ImVec2> points, velocities;
	float filterAlphaPos = 0.5;

	// Load Fonts
	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->AddFontDefault();
	ui::init(20.);

	arma_rng::set_seed_random();

	int nbData = 100; //Number of datapoints

	int nbVarTan = 3 + 3; //Dimension of tangent space data (here: TE3 + TS3)
	int nbDeriv = 2; //Number of static & dynamic features (D=2 for [x,dx])
	int nbVar = nbVarTan * nbDeriv; //Dimension of tangent state space
	int nbVarMan = nbVarTan + 1; //Dimension of the manifold (here: E3 + S3)
	double params_diagRegFact = 1E-4; //Regularization of covariance
	double dt = 1E-2; //Time step duration
	double rfactor = 1E-1;	//Control cost in LQR
	int nbRepros = 5;
	int nbRepros_prev = 5;

	//Control cost matrix
	mat R = eye(nbVarTan, nbVarTan) * rfactor;
	//R.submat(0, 0, 2, 2)=diagmat(ones(3,1)*1E-1); % Set different position control cost

	//Target and desired covariance
	colvec xTar;
	xTar = (randn(4)) * 3;
	xTar = xTar / norm(xTar, 2);
	xTar = join_vert(randn(3), xTar);

	mat uCov = eye(nbVarTan, nbVarTan) * 1E-3;

	//Eigendecomposition
	vec tmpEigenValues;
	mat tmpEigenVectors;
	eig_sym(tmpEigenValues, tmpEigenVectors, uCov);
	mat U0 = tmpEigenVectors * diagmat(pow(tmpEigenValues, 0.5));

	mat A = eye(nbVarTan * 2, nbVarTan * 2);
	A.submat(0, nbVarTan, nbVarTan - 1, (nbVarTan * 2) - 1) = eye(nbVarTan, nbVarTan) * dt;
	mat B = join_vert(eye(nbVarTan, nbVarTan) * pow(dt, 2), eye(nbVarTan, nbVarTan) * dt);

	colvec duTar = zeros(nbVarTan * (nbDeriv - 1), 1);

	bool recompute = true;
	bool init = true;
	cube Xsave(7, nbData, nbRepros);

	while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();

		// Detect when the window was resized
		if ((ImGui::GetIO().DisplaySize.x != window_size.win_width) ||
			(ImGui::GetIO().DisplaySize.y != window_size.win_height)) {

			// Retrieve the new window size
			glfwGetWindowSize(window, &window_size.win_width, &window_size.win_height);

			// Retrieve the new framebuffer size
			glfwGetFramebufferSize(window, &window_size.fb_width, &window_size.fb_height);
		}

		ImGui_ImplGlfwGL2_NewFrame();

		if (init)
			ImGui::SetNextWindowPos(ImVec2(450, 430));

		ui::begin("demo");
		ImGuiWindow *win = ImGui::GetCurrentWindow();
		ui::end();

		if (init) { // Init the control panel
			ImGui::SetNextWindowPos(ImVec2(2, 2));
			ImGui::SetNextWindowCollapsed(false); //true);
		}

		nbRepros_prev = nbRepros;
		ImGui::SetNextWindowSize(ImVec2(400, 54));
		ImGui::Begin("Parameters", NULL, ImGuiWindowFlags_NoResize |
					 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove);
		ImGui::SliderInt("Nb repros", &nbRepros, 1, 10);

		ImVec4 clip_rect2(0, 0, window_size.win_width, window_size.win_height);

		// time
		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(window_size.win_width / 4 - 20, window_size.win_height - 40),
							   ImColor(0, 0, 0, 255), "time",
							   NULL, 0.0f, &clip_rect2);

		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(3 * window_size.win_width / 4 - 20, window_size.win_height - 40),
							   ImColor(0, 0, 0, 255), "time",
							   NULL, 0.0f, &clip_rect2);

		// xyz
		int plot_height_px = (window_size.win_height - 50) / 4 - 50;

		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(20, 3 * (plot_height_px + 50) + 50 + plot_height_px / 2 - 10),
							   ImColor(0, 0, 0, 255), "z", NULL, 0.0f, &clip_rect2);

		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(20, 2 * (plot_height_px + 50) + 50 + plot_height_px / 2 - 10),
							   ImColor(0, 0, 0, 255), "y", NULL, 0.0f, &clip_rect2);

		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(20, (plot_height_px + 50) + 50 + plot_height_px / 2 - 10),
							   ImColor(0, 0, 0, 255), "x", NULL, 0.0f, &clip_rect2);

		// qw qx qy qz
		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(window_size.win_width / 2 + 10,
									  3 * (plot_height_px + 50) + 50 + plot_height_px / 2 - 10),
							   ImColor(0, 0, 0, 255), "qz", NULL, 0.0f, &clip_rect2);

		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(window_size.win_width / 2 + 10,
									  2 * (plot_height_px + 50) + 50 + plot_height_px / 2 - 10),
							   ImColor(0, 0, 0, 255), "qy", NULL, 0.0f, &clip_rect2);

		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(window_size.win_width / 2 + 10,
									  (plot_height_px + 50) + 50 + plot_height_px / 2 - 10),
							   ImColor(0, 0, 0, 255), "qx", NULL, 0.0f, &clip_rect2);

		win->DrawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 1.5f,
							   ImVec2(window_size.win_width / 2 + 10,
									  50 + plot_height_px / 2 - 10),
							   ImColor(0, 0, 0, 255), "qw", NULL, 0.0f, &clip_rect2);

		ImGui::End();

		// Recompute if needed
		if (init || (nbRepros_prev != nbRepros)) {

				xTar = (randn(4)) * 3;
				xTar = xTar / norm(xTar, 2);
				xTar = join_vert(randn(3), xTar);
			Xsave.clear();
			Xsave = zeros(7, nbData, nbRepros);
			for (int r = 0; r < nbRepros; r++) {

				vec x;
				x << 0 << 0 << 0 << -1 << -1 << 1 << 0;

				x = x + randn(nbVarMan) * 1.5;

				vec tmpVec = { 0, 0, 0 };
				//x.subvec(0, 2) = tmpVec; // Start position
				x.subvec(3, 6) = x.subvec(3, 6) / norm(x.subvec(3, 6), 2);
				vec x_old = x;
				vec U = zeros(nbVar, 1);

				mat Ac;
				mat U1;
				mat uCovTmp;
				mat Q = zeros(nbVar, nbVar);
				mat P, L;
				vec ddu;


				for (int t = 0; t < nbData; t++) {
					Xsave.slice(r).col(t) = x;
					U.subvec(0, nbVarTan - 1) = zeros(nbVarTan, 1); //Set tangent space at x

					//Transportation of velocity vector from x_old to x
					if (norm(x_old - x, 2) != 0) {					// haven't reached target
						Ac = transp(x_old, x);
					} else {
						Ac = eye(nbVarTan, nbVarTan);
					}

					U.subvec(nbVarTan, (2 * nbVarTan) - 1) = Ac * U.subvec(nbVarTan, (2 * nbVarTan) - 1); //Transport of velocity
					U.subvec(0, 2) = x.subvec(0, 2); // Transport of position (action part)

					//Transportation of uCov from xTar to x
					Ac = transp(xTar, x);
					if (norm(xTar - x, 2) != 0) {					// haven't reached target
						Ac = transp(xTar, x);
					} else {
						Ac = eye(nbVarTan, nbVarTan);
					}
					U1 = Ac * U0;
					uCovTmp = U1 * U1.t();

					Q = zeros(nbVar, nbVar);
					Q.submat(0, 0, nbVarTan - 1, nbVarTan - 1) = inv(uCovTmp); //Precision matrix
					P = solveAlgebraicRiccati_eig_discrete(A, B, Q, R); // * (inv(R) * B.t()), (Q + Q.t()) / 2);


					L = inv(B.t() * P * B + R) * B.t() * P * A; //Feedback gain (discrete version)
					ddu = L * (join_vert(logmap(xTar, x), duTar) - U); //Compute acceleration
					U = A * U + B * ddu; //Update U
					x_old = x;

					x = expmap(U.subvec(0, nbVarTan - 1), x);
				}

				if (all((Xsave.slice(r).col(nbData - 1).subvec(3, 6) % xTar.subvec(3, 6)) < 0)) {
					Xsave.slice(r).rows(3, 6) = Xsave.slice(r).rows(3, 6) * -1.0;
				}

			}
			recompute = false;
		}

		//======================== GUI rendering ========================
		glViewport(0, 0, window_size.fb_width, window_size.fb_height);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear (GL_COLOR_BUFFER_BIT);
		ImGui::Render();
		ImGui_ImplGlfwGL2_RenderDrawData(ImGui::GetDrawData());

		//======================== PbDlib rendering ========================
		glPushMatrix();
		glTranslatef(-1.0f, -1.0f, 0);
		glScalef(2.0f / (float) window_size.fb_width,
				 2.0f / (float) window_size.fb_height, 1.0f);
		glLineWidth(2.0f);

		//======================== PLOTTING ========================
		float lineWidth = 1.5;
		float biasX = 0.0;
		float biasY = 0.0;
		float scalerX = 1.0;
		float scalerY = 1.0;
		float min_value = 0.0;

		float max_xyz = std::max(Xsave(span(0, 2), span::all, span::all).max(), xTar(span(0, 2)).max());
		float min_xyz = std::min(Xsave(span(0, 2), span::all, span::all).min(), xTar(span(0, 2)).min());
		float max_q = std::max(Xsave(span(3, 6), span::all, span::all).max(), xTar(span(3, 6)).max());
		float min_q = std::min(Xsave(span(3, 6), span::all, span::all).min(), xTar(span(3, 6)).min());

		float plot_width = (window_size.fb_width / 2 - 100 * window_size.scale_x());
		float plot_height = (window_size.fb_height - 50 * window_size.scale_y()) / 4 -
							50 * window_size.scale_y();

		for (int r = 0; r < nbRepros; r++) {

			for (int i = 0; i < 7; i++) {
				scalerX = plot_width / nbData;

				if (i < 3) {
					biasX = 50.0 * window_size.scale_x();
					biasY = (2 - i) * (plot_height + 50 * window_size.scale_y()) +
							50 * window_size.scale_y();

					scalerY = plot_height / (max_xyz - min_xyz);
					min_value = min_xyz;

				} else {
					biasX = window_size.fb_width / 2 + 50.0 * window_size.scale_x();

					biasY = (6 - i) * (plot_height + 50 * window_size.scale_y()) +
							50 * window_size.scale_y();

					scalerY = plot_height / (max_q - min_q);
					min_value = min_q;
				}

				if (r == 0) { //plot the background
					glColor3f(0.9f, 0.9f, 0.9f);

					glBegin (GL_QUADS);
					glVertex2f(biasX, biasY);
					glVertex2f(biasX + plot_width, biasY);
					glVertex2f(biasX + plot_width, biasY + plot_height);
					glVertex2f(biasX, biasY + plot_height);
					glEnd();
				}

				glLineWidth(lineWidth);
				glColor3f(COLORS(r, 0), COLORS(r, 1), COLORS(r, 2));

				glBegin (GL_LINE_STRIP);
				float dummyx;
				float dummyy;
				for (int t = 0; t < Xsave.n_cols - 1; t++) {
					dummyx = t * scalerX + biasX;
					dummyy = (float) (Xsave.slice(r)(i, t) - min_value) * scalerY + biasY;
					glVertex2f(dummyx, dummyy);
				}
				glEnd();

				if (r == 0) { //plot the reference
					glLineWidth(lineWidth);
					glColor3f(0.0f, 0.0f, 0.0f);
					glBegin(GL_LINE_STRIP);
					float dummyx;
					float dummyy;
					for (int t = 0; t < Xsave.n_cols - 1; t++) {
						dummyx = t * scalerX + biasX;
						dummyy = (xTar(i) - min_value) * scalerY + biasY;
						glVertex2f(dummyx, dummyy);
					}
					glEnd();
				}
			}
		}

		glPopMatrix();
		glfwSwapBuffers(window);

		// Keyboard input
		if (ImGui::IsKeyPressed(GLFW_KEY_ESCAPE))
			break;

		init = false;
	}

	ImGui_ImplGlfwGL2_Shutdown();
	glfwTerminate();

	return 0;
}
