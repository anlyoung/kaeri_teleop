/*
 * gfx3.cpp
 *
 * Rendering utility structures and functions based on OpenGL 3.3+
 *
 * Authors: Philip Abbet
 */

#include <gfx2.h>

namespace gfx2 {


/****************************** UTILITY FUNCTIONS ****************************/

void init()
{
	glewExperimental = GL_TRUE;
	glewInit();
}

//-----------------------------------------------

double deg2rad(double deg)
{
	return deg / 360.0 * (2.0 * M_PI);
}

//-----------------------------------------------

double sin_deg(double deg)
{
	return sin(deg2rad(deg));
}

//-----------------------------------------------

double cos_deg(double deg)
{
	return cos(deg2rad(deg));
}

//-----------------------------------------------

bool is_close(float a, float b, float epsilon)
{
	return fabs(a - b) < epsilon;
}

//-----------------------------------------------

arma::vec ui2fb(const arma::vec& coords, int win_width, int win_height,
				int fb_width, int fb_height) {
	arma::vec result = coords;

	result(0) = coords(0) * (float) fb_width / (float) win_width;
	result(1) = ((float) win_height - coords(1)) * (float) fb_height / (float) win_height;

	return result;
}

//-----------------------------------------------

arma::vec ui2fb(const arma::vec& coords, const window_size_t& window_size) {
	arma::vec result = coords;

	result(0) = coords(0) * (float) window_size.fb_width / (float) window_size.win_width;

	result(1) = ((float) window_size.win_height - coords(1)) *
				(float) window_size.fb_height / (float) window_size.win_height;

	return result;
}

//-----------------------------------------------

arma::vec ui2fb_centered(const arma::vec& coords, int win_width, int win_height,
						 int fb_width, int fb_height) {
	arma::vec result = coords;

	result(0) = (coords(0) - (float) win_width * 0.5f) * (float) fb_width / (float) win_width;
	result(1) = ((float) win_height * 0.5f - coords(1)) * (float) fb_height / (float) win_height;

	return result;
}

//-----------------------------------------------

arma::vec ui2fb_centered(const arma::vec& coords, const window_size_t& window_size) {
	arma::vec result = coords;

	result(0) = (coords(0) - (float) window_size.win_width * 0.5f) *
				(float) window_size.fb_width / (float) window_size.win_width;

	result(1) = ((float) window_size.win_height * 0.5f - coords(1)) *
				(float) window_size.fb_height / (float) window_size.win_height;

	return result;
}

//-----------------------------------------------

arma::vec fb2ui(const arma::vec& coords, int win_width, int win_height,
				int fb_width, int fb_height) {
	arma::vec result = coords;

	result(0) = coords(0) * (float) win_width / (float) fb_width;
	result(1) = -(coords(1) * (float) win_height / (float) fb_height - (float) win_height);

	return result;
}

//-----------------------------------------------

arma::vec fb2ui(const arma::vec& coords, const window_size_t& window_size) {
	arma::vec result = coords;

	result(0) = coords(0) * (float) window_size.win_width / (float) window_size.fb_width;
	result(1) = -(coords(1) * (float) window_size.win_height / (float) window_size.fb_height -
				(float) window_size.win_height);

	return result;
}

//-----------------------------------------------

arma::vec fb2ui_centered(const arma::vec& coords, int win_width, int win_height,
						 int fb_width, int fb_height) {
	arma::vec result = coords;

	result(0) = coords(0) * (float) win_width / (float) fb_width + (float) win_width * 0.5f;
	result(1) = -(coords(1) * (float) win_height / (float) fb_height - (float) win_height * 0.5f);

	return result;
}

//-----------------------------------------------

arma::vec fb2ui_centered(const arma::vec& coords, const window_size_t& window_size) {
	arma::vec result = coords;

	result(0) = coords(0) * (float) window_size.win_width / (float) window_size.fb_width +
				(float) window_size.win_width * 0.5f;
	result(1) = -(coords(1) * (float) window_size.win_height / (float) window_size.fb_height -
				(float) window_size.win_height * 0.5f);

	return result;
}


/************************* PROJECTION & VIEW MATRICES ************************/

arma::fmat perspective(float fovy, float aspect, float zNear, float zFar)
{
	const float top = zNear * tan(fovy / 2.0f);
	const float bottom = -top;
	const float right = top * aspect;
	const float left = -right;

	arma::fmat projection = arma::zeros<arma::fmat>(4,4);

	projection(0, 0) = 2.0f * zNear / (right - left);
	projection(0, 2) = (right + left) / (right - left);
	projection(1, 1) = 2.0f * zNear / (top - bottom);
	projection(1, 2) = (top + bottom) / (top - bottom);
	projection(2, 2) = -(zFar + zNear) / (zFar - zNear);
	projection(2, 3) = -(2.0f * zFar * zNear) / (zFar - zNear);
	projection(3, 2) = -1.0f;

	return projection;
}

//-----------------------------------------------

arma::fmat lookAt(const arma::fvec& position, const arma::fvec& target,
				  const arma::fvec& up)
{
	const arma::fvec f(arma::normalise(target - position));
	const arma::fvec s(arma::normalise(arma::cross(f, up)));
	const arma::fvec u(arma::cross(s, f));

	arma::fmat result = arma::zeros<arma::fmat>(4,4);

	result(0, 0) = s(0);
	result(0, 1) = s(1);
	result(0, 2) = s(2);
	result(1, 0) = u(0);
	result(1, 1) = u(1);
	result(1, 2) = u(2);
	result(2, 0) =-f(0);
	result(2, 1) =-f(1);
	result(2, 2) =-f(2);
	result(0, 3) =-arma::dot(s, position);
	result(1, 3) =-arma::dot(u, position);
	result(2, 3) = arma::dot(f, position);
	result(3, 3) = 1.0f;

	return result;
}


/****************************** TRANSFORMATIONS ******************************/

arma::fmat rotate(const arma::fvec& axis, float angle)
{
	float rcos = cos(angle);
	float rsin = sin(angle);

	arma::fmat matrix = arma::zeros<arma::fmat>(4, 4);

	matrix(0, 0) =			  rcos + axis(0) * axis(0) * (1.0f - rcos);
	matrix(1, 0) =	axis(2) * rsin + axis(1) * axis(0) * (1.0f - rcos);
	matrix(2, 0) = -axis(1) * rsin + axis(2) * axis(0) * (1.0f - rcos);
	matrix(0, 1) = -axis(2) * rsin + axis(0) * axis(1) * (1.0f - rcos);
	matrix(1, 1) =			  rcos + axis(1) * axis(1) * (1.0f - rcos);
	matrix(2, 1) =	axis(0) * rsin + axis(2) * axis(1) * (1.0f - rcos);
	matrix(0, 2) =	axis(1) * rsin + axis(0) * axis(2) * (1.0f - rcos);
	matrix(1, 2) = -axis(0) * rsin + axis(1) * axis(2) * (1.0f - rcos);
	matrix(2, 2) =			  rcos + axis(2) * axis(2) * (1.0f - rcos);
	matrix(3, 3) = 1.0f;

	return matrix;
}

//-----------------------------------------------

arma::fmat rotation(const arma::fvec& from, const arma::fvec& to)
{
	const float dot = arma::dot(from, to);
	const arma::fvec cross = arma::cross(from, to);
	const float norm = arma::norm(cross);

	arma::fmat g({
		{ dot,	-norm, 0.0f },
		{ norm,	 dot,  0.0f },
		{ 0.0f,	 0.0f, 1.0f },
	});

	arma::fmat fi(3, 3);
	fi.rows(0, 0) = from.t();
	fi.rows(1, 1) = arma::normalise(to - dot * from).t();
	fi.rows(2, 2) = arma::cross(to, from).t();

	arma::fmat result = arma::eye<arma::fmat>(4, 4);

	arma::fmat u;
	if (arma::inv(u, fi))
	{
		u = u * g * fi;
		result.submat(0, 0, 2, 2) = u;
	}

	return result;
}

//-----------------------------------------------

void rigid_transform_3D(const arma::fmat& A, const arma::fmat& B,
						arma::fmat &rotation, arma::fvec &translation) {

	arma::fvec centroidsA = arma::mean(A, 1);
	arma::fvec centroidsB = arma::mean(B, 1);

	int n = A.n_cols;

	arma::fmat H = (A - repmat(centroidsA, 1, n)) * (B - repmat(centroidsB, 1, n)).t();

	arma::fmat U, V;
	arma::fvec s;
	arma::svd(U, s, V, H);

	rotation = V * U.t();

	if (arma::det(rotation) < 0.0f)
		rotation.col(2) *= -1.0f;

	translation = -rotation * centroidsA + centroidsB;
}

//-----------------------------------------------

arma::fmat worldTransforms(const transforms_t* transforms)
{
	arma::fmat result = arma::eye<arma::fmat>(4, 4);
	result(0, 3, arma::size(3, 1)) = worldPosition(transforms);

	result = result * worldRotation(transforms);

	return result;
}

//-----------------------------------------------

arma::fvec worldPosition(const transforms_t* transforms)
{
	if (transforms->parent)
	{
		arma::fvec position(4);
		position.rows(0, 2) = transforms->position;
		position(3) = 1.0f;

		position = worldRotation(transforms->parent) * position;

		return worldPosition(transforms->parent) + position.rows(0, 2);
	}

	return transforms->position;
}

//-----------------------------------------------

arma::fmat worldRotation(const transforms_t* transforms)
{
	if (transforms->parent)
	{
		arma::fmat result = worldRotation(transforms->parent) * transforms->rotation;
		return arma::normalise(result);
	}

	return transforms->rotation;
}


/********************************** TEXTURES *********************************/

texture_t create_texture(int width, int height, GLenum format, GLenum type)
{
	texture_t texture = { 0 };

	texture.width = (GLuint) width;
	texture.height = (GLuint) height;
	texture.format = format;
	texture.type = type;

	glGenTextures(1, &texture.id);

	glBindTexture(GL_TEXTURE_2D, texture.id);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	if (texture.type == GL_FLOAT)
		texture.pixels_f = new float[width * height * 3];
	else
		texture.pixels_b = new unsigned char[width * height * 3];

	return texture;
}

//-----------------------------------------------

void destroy(texture_t &texture)
{
	if (texture.type == GL_FLOAT)
		delete[] texture.pixels_f;
	else
		delete[] texture.pixels_b;

	glDeleteTextures(1, &texture.id);

	texture = {0};
}


/*********************************** MESHES **********************************/

model_t create_rectangle(const arma::fvec& color, float width, float height,
						 const arma::fvec& position, const arma::fmat& rotation,
						 transforms_t* parent_transforms)
{
	model_t model = { 0 };

	model.mode = GL_TRIANGLES;
	model.lightning_enabled = false;
	model.use_one_minus_src_alpha_blending = false;

	// Position & rotation
	model.transforms.position = position;
	model.transforms.rotation = rotation;
	model.transforms.parent = parent_transforms;

	// Material
	model.diffuse_color = color;

	// Create the mesh
	model.nb_vertices = 6;

	//-- Vertex buffer
	float half_width = 0.5f * width;
	float half_height = 0.5f * height;

	const GLfloat vertex_buffer_data[] = {
		 half_width,  half_height, 0.0f,
		-half_width,  half_height, 0.0f,
		-half_width, -half_height, 0.0f,
		-half_width, -half_height, 0.0f,
		 half_width, -half_height, 0.0f,
		 half_width,  half_height, 0.0f,
	};

	model.vertex_buffer = new GLfloat[model.nb_vertices * 3];
	memcpy(model.vertex_buffer, vertex_buffer_data,
		   model.nb_vertices * 3 * sizeof(GLfloat));

	//-- UVs buffer
	const GLfloat uv_buffer_data[] = {
		1.0f, 1.0f,
		0.0f, 1.0f,
		0.0f, 0.0f,
		0.0f, 0.0f,
		1.0f, 0.0f,
		1.0f, 1.0f,
	};

	model.uv_buffer = new GLfloat[model.nb_vertices * 2];
	memcpy(model.uv_buffer, uv_buffer_data, model.nb_vertices * 2 * sizeof(GLfloat));

	return model;
}

//-----------------------------------------------

model_t create_rectangle(const texture_t& texture, float width, float height,
						 const arma::fvec& position, const arma::fmat& rotation,
						 transforms_t* parent_transforms)
{
	model_t model = create_rectangle(arma::fvec({1.0f, 1.0f, 1.0f, 1.0f}),
									 width, height, position, rotation,
						 			 parent_transforms);

	model.texture = texture;

	return model;
}

//-----------------------------------------------

model_t create_square(const arma::fvec& color, float size, const arma::fvec& position,
					  const arma::fmat& rotation, transforms_t* parent_transforms)
{
	model_t model = { 0 };

	model.mode = GL_TRIANGLES;
	model.lightning_enabled = false;
	model.use_one_minus_src_alpha_blending = false;

	// Position & rotation
	model.transforms.position = position;
	model.transforms.rotation = rotation;
	model.transforms.parent = parent_transforms;

	// Material
	model.diffuse_color = color;

	// Create the mesh
	model.nb_vertices = 6;

	//-- Vertex buffer
	float half_size = 0.5f * size;

	const GLfloat vertex_buffer_data[] = {
		 half_size,	 half_size, 0.0f,
		-half_size,	 half_size, 0.0f,
		-half_size, -half_size, 0.0f,
		-half_size, -half_size, 0.0f,
		 half_size, -half_size, 0.0f,
		 half_size,	 half_size, 0.0f,
	};

	model.vertex_buffer = new GLfloat[model.nb_vertices * 3];
	memcpy(model.vertex_buffer, vertex_buffer_data,
		   model.nb_vertices * 3 * sizeof(GLfloat));

	//-- UVs buffer
	const GLfloat uv_buffer_data[] = {
		1.0f, 1.0f,
		0.0f, 1.0f,
		0.0f, 0.0f,
		0.0f, 0.0f,
		1.0f, 0.0f,
		1.0f, 1.0f,
	};

	model.uv_buffer = new GLfloat[model.nb_vertices * 2];
	memcpy(model.uv_buffer, uv_buffer_data, model.nb_vertices * 2 * sizeof(GLfloat));

	return model;
}

//-----------------------------------------------

model_t create_sphere(float radius, const arma::fvec& position,
					  const arma::fmat& rotation, transforms_t* parent_transforms)
{
	model_t model = { 0 };

	model.mode = GL_TRIANGLES;
	model.lightning_enabled = true;
	model.use_one_minus_src_alpha_blending = false;

	// Position & rotation
	model.transforms.position = position;
	model.transforms.rotation = rotation;
	model.transforms.parent = parent_transforms;

	// Material
	model.ambiant_color = arma::fvec({0.2f, 0.2f, 0.2f});
	model.diffuse_color = arma::fvec({0.8f, 0.8f, 0.8f});
	model.specular_color = arma::fvec({0.0f, 0.0f, 0.0f});
	model.specular_power = 5;

	// Create the mesh
	const int NB_STEPS = 72;
	const float STEP_SIZE = 360.0f / NB_STEPS;

	model.nb_vertices = NB_STEPS / 2 * NB_STEPS * 6;

	model.vertex_buffer = new GLfloat[model.nb_vertices * 3];
	model.normal_buffer = new GLfloat[model.nb_vertices * 3];

	GLfloat* dst_vertex = model.vertex_buffer;
	GLfloat* dst_normal = model.normal_buffer;

	for (int i = 0; i < NB_STEPS / 2; ++i)
	{
		GLfloat latitude_lo = (float) i * STEP_SIZE;
		GLfloat latitude_hi = latitude_lo + STEP_SIZE;

		for (int j = 0; j < NB_STEPS; ++j)
		{
			GLfloat longitude_lo = (float) j * STEP_SIZE;
			GLfloat longitude_hi = longitude_lo + STEP_SIZE;

			arma::fvec vert_ne(3);
			arma::fvec vert_nw(3);
			arma::fvec vert_sw(3);
			arma::fvec vert_se(3);

			// Assign each X,Z with sin,cos values scaled by latitude radius indexed by longitude.
			vert_ne(1) = vert_nw(1) = (float) -cos_deg(latitude_hi) * radius;
			vert_sw(1) = vert_se(1) = (float) -cos_deg(latitude_lo) * radius;

			vert_nw(0) = (float) cos_deg(longitude_lo) * (radius * (float) sin_deg(latitude_hi));
			vert_sw(0) = (float) cos_deg(longitude_lo) * (radius * (float) sin_deg(latitude_lo));
			vert_ne(0) = (float) cos_deg(longitude_hi) * (radius * (float) sin_deg(latitude_hi));
			vert_se(0) = (float) cos_deg(longitude_hi) * (radius * (float) sin_deg(latitude_lo));

			vert_nw(2) = (float) -sin_deg(longitude_lo) * (radius * (float) sin_deg(latitude_hi));
			vert_sw(2) = (float) -sin_deg(longitude_lo) * (radius * (float) sin_deg(latitude_lo));
			vert_ne(2) = (float) -sin_deg(longitude_hi) * (radius * (float) sin_deg(latitude_hi));
			vert_se(2) = (float) -sin_deg(longitude_hi) * (radius * (float) sin_deg(latitude_lo));

			dst_vertex[0] = vert_ne(0); dst_vertex[1] = vert_ne(1); dst_vertex[2] = vert_ne(2); dst_vertex += 3;
			dst_vertex[0] = vert_nw(0); dst_vertex[1] = vert_nw(1); dst_vertex[2] = vert_nw(2); dst_vertex += 3;
			dst_vertex[0] = vert_sw(0); dst_vertex[1] = vert_sw(1); dst_vertex[2] = vert_sw(2); dst_vertex += 3;

			dst_vertex[0] = vert_sw(0); dst_vertex[1] = vert_sw(1); dst_vertex[2] = vert_sw(2); dst_vertex += 3;
			dst_vertex[0] = vert_se(0); dst_vertex[1] = vert_se(1); dst_vertex[2] = vert_se(2); dst_vertex += 3;
			dst_vertex[0] = vert_ne(0); dst_vertex[1] = vert_ne(1); dst_vertex[2] = vert_ne(2); dst_vertex += 3;

			// Normals
			arma::fvec normal_ne = arma::normalise(vert_ne);
			arma::fvec normal_nw = arma::normalise(vert_nw);
			arma::fvec normal_sw = arma::normalise(vert_sw);
			arma::fvec normal_se = arma::normalise(vert_se);

			dst_normal[0] = normal_ne(0); dst_normal[1] = normal_ne(1); dst_normal[2] = normal_ne(2); dst_normal += 3;
			dst_normal[0] = normal_nw(0); dst_normal[1] = normal_nw(1); dst_normal[2] = normal_nw(2); dst_normal += 3;
			dst_normal[0] = normal_sw(0); dst_normal[1] = normal_sw(1); dst_normal[2] = normal_sw(2); dst_normal += 3;

			dst_normal[0] = normal_sw(0); dst_normal[1] = normal_sw(1); dst_normal[2] = normal_sw(2); dst_normal += 3;
			dst_normal[0] = normal_se(0); dst_normal[1] = normal_se(1); dst_normal[2] = normal_se(2); dst_normal += 3;
			dst_normal[0] = normal_ne(0); dst_normal[1] = normal_ne(1); dst_normal[2] = normal_ne(2); dst_normal += 3;
		}
	}

	return model;
}

//-----------------------------------------------

model_t create_line(const arma::fvec& color, const arma::mat& points,
					const arma::fvec& position, const arma::fmat& rotation,
					transforms_t* parent_transforms, bool line_strip)
{
	model_t model = { 0 };

	model.mode = (line_strip ? GL_LINE_STRIP : GL_LINES);
	model.lightning_enabled = false;
	model.use_one_minus_src_alpha_blending = false;

	// Position & rotation
	model.transforms.position = position;
	model.transforms.rotation = rotation;
	model.transforms.parent = parent_transforms;

	// Material
	model.diffuse_color = color;

	// Create the mesh
	model.nb_vertices = points.n_cols;

	model.vertex_buffer = new GLfloat[model.nb_vertices * 3];

	GLfloat* dst = model.vertex_buffer;

	for (int i = 0; i < points.n_cols; ++i) {
		dst[0] = (float) points(0, i);
		dst[1] = (float) points(1, i);

		if (points.n_rows == 3)
			dst[2] = (float) points(2, i);
		else
			dst[2] = 0.0f;

		dst += 3;
	}

	return model;
}

//-----------------------------------------------

model_t create_line(const arma::fvec& color, const std::vector<arma::vec>& points,
					const arma::fvec& position, const arma::fmat& rotation,
					transforms_t* parent_transforms, bool line_strip)
{
	arma::mat points_mat(points[0].n_rows, points.size());

	for (size_t i = 0; i < points.size(); ++i)
		points_mat.col(i) = points[i];

	return create_line(color, points_mat, position, rotation, parent_transforms, line_strip);
}

//-----------------------------------------------

model_t create_mesh(const arma::fvec& color, const arma::mat& vertices,
					const arma::fvec& position, const arma::fmat& rotation,
					transforms_t* parent_transforms)
{
	model_t model = { 0 };

	model.mode = GL_TRIANGLES;
	model.lightning_enabled = false;
	model.use_one_minus_src_alpha_blending = false;

	// Position & rotation
	model.transforms.position = position;
	model.transforms.rotation = rotation;
	model.transforms.parent = parent_transforms;

	// Material
	model.diffuse_color = color;

	// Create the mesh
	model.nb_vertices = vertices.n_cols;

	model.vertex_buffer = new GLfloat[model.nb_vertices * 3];

	GLfloat* dst = model.vertex_buffer;

	for (int i = 0; i < vertices.n_cols; ++i) {
		dst[0] = (float) vertices(0, i);
		dst[1] = (float) vertices(1, i);

		if (vertices.n_rows == 3)
			dst[2] = (float) vertices(2, i);
		else
			dst[2] = 0.0f;

		dst += 3;
	}

	return model;
}

//-----------------------------------------------

model_t create_gaussian_background(const arma::fvec& color, const arma::vec& mu,
								   const arma::mat& sigma,
								   const arma::fvec& position, const arma::fmat& rotation,
								   transforms_t* parent_transforms)
{
	arma::mat vertices = get_gaussian_background_vertices(mu, sigma, 60);

	model_t model = create_mesh(color, vertices, position, rotation, parent_transforms);

	model.use_one_minus_src_alpha_blending = true;

	return model;
}

//-----------------------------------------------

model_t create_gaussian_border(const arma::fvec& color, const arma::vec& mu,
							   const arma::mat& sigma,
							   const arma::fvec& position, const arma::fmat& rotation,
							   transforms_t* parent_transforms)
{
	arma::mat pts = get_gaussian_border_vertices(mu, sigma, 60, true);

	return create_line(color, pts, position, rotation, parent_transforms);
}

//-----------------------------------------------

void destroy(model_t &model)
{
	if (model.vertex_buffer)
		delete[] model.vertex_buffer;

	if (model.normal_buffer)
		delete[] model.normal_buffer;

	if (model.uv_buffer)
		delete[] model.uv_buffer;

	model = {0};
}


/********************************** RENDERING ********************************/

bool draw(const model_t& model, const light_list_t& lights)
{
	// Various checks
	if (model.lightning_enabled && lights.empty())
		return false;

	// Specify material parameters for the lighting model
	if (model.lightning_enabled) {
		glMaterialfv(GL_FRONT, GL_AMBIENT, model.ambiant_color.memptr());
		glMaterialfv(GL_FRONT, GL_DIFFUSE, model.diffuse_color.memptr());
		glMaterialfv(GL_FRONT, GL_SPECULAR, model.specular_color.memptr());
		glMaterialf(GL_FRONT, GL_SHININESS, model.specular_power);
	} else {
		if (model.texture.width > 0)
			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		else if (model.diffuse_color.n_rows == 3)
			glColor3fv(model.diffuse_color.memptr());
		else
			glColor4fv(model.diffuse_color.memptr());
	}

	// Specify the light parameters
	glDisable(GL_LIGHTING);
	if (model.lightning_enabled) {
		glEnable(GL_LIGHTING);

		glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_POSITION, lights[0].transforms.position.memptr());
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lights[0].color.memptr());
	}

	// Set vertex data
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, model.vertex_buffer);

	// Set normal data
	if (model.lightning_enabled) {
		glEnableClientState(GL_NORMAL_ARRAY);
		glNormalPointer(GL_FLOAT, 0, model.normal_buffer);
	}

	// Set UV data
	if (model.uv_buffer) {
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glTexCoordPointer(2, GL_FLOAT, 0, model.uv_buffer);
	}

	// Texturing
	if (model.texture.width > 0) {
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, model.texture.id);

		if (model.texture.type == GL_FLOAT) {
			glTexImage2D(GL_TEXTURE_2D, 0, model.texture.format,
						 model.texture.width, model.texture.height,
						 0, model.texture.format, GL_FLOAT,
						 model.texture.pixels_f
			);
		} else {
			glTexImage2D(GL_TEXTURE_2D, 0, model.texture.format,
						 model.texture.width, model.texture.height,
						 0, model.texture.format, GL_UNSIGNED_BYTE,
						 model.texture.pixels_b
			);
		}
	}

	// Apply the model matrix
	arma::fmat model_matrix = worldTransforms(&model.transforms);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(model_matrix.memptr());

	GLboolean is_blending_enabled = glIsEnabled(GL_BLEND);
	GLint blend_src_rgb, blend_src_alpha, blend_dst_rgb, blend_dst_alpha;

	if (model.use_one_minus_src_alpha_blending)
	{
		if (is_blending_enabled)
		{
			glGetIntegerv(GL_BLEND_SRC_RGB, &blend_src_rgb);
			glGetIntegerv(GL_BLEND_SRC_ALPHA, &blend_src_alpha);
			glGetIntegerv(GL_BLEND_DST_RGB, &blend_dst_rgb);
			glGetIntegerv(GL_BLEND_DST_ALPHA, &blend_dst_alpha);
		}
		else
		{
			glEnable(GL_BLEND);
		}

		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	// Draw the mesh
	glDrawArrays(model.mode, 0, model.nb_vertices);

	if (model.use_one_minus_src_alpha_blending)
	{
		if (is_blending_enabled)
		{
			glBlendFunc(GL_SRC_COLOR, blend_src_rgb);
			glBlendFunc(GL_SRC_ALPHA, blend_src_alpha);
			glBlendFunc(GL_DST_COLOR, blend_dst_rgb);
			glBlendFunc(GL_DST_ALPHA, blend_dst_alpha);
		}
		else
		{
			glDisable(GL_BLEND);
		}
	}

	glPopMatrix();

	// Cleanup
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	if (model.texture.width > 0)
		glDisable(GL_TEXTURE_2D);

	return true;
}

//-----------------------------------------------

bool draw_rectangle(const arma::fvec& color, float width, float height,
					const arma::fvec& position, const arma::fmat& rotation)
{
	model_t rect = create_rectangle(color, width, height, position, rotation);

	bool result = draw(rect);

	destroy(rect);

	return result;
}

//-----------------------------------------------

bool draw_rectangle(const texture_t& texture, float width, float height,
					const arma::fvec& position, const arma::fmat& rotation)
{
	model_t rect = create_rectangle(texture, width, height, position, rotation);

	bool result = draw(rect);

	destroy(rect);

	return result;
}

//-----------------------------------------------

bool draw_line(const arma::fvec& color, const arma::mat& points,
			   const arma::fvec& position, const arma::fmat& rotation)
{
	model_t line = create_line(color, points, position, rotation);

	bool result = draw(line);

	destroy(line);

	return result;
}

//-----------------------------------------------

bool draw_line(const arma::fvec& color, const std::vector<arma::vec>& points,
			   const arma::fvec& position, const arma::fmat& rotation)
{
	model_t line = create_line(color, points, position, rotation);

	bool result = draw(line);

	destroy(line);

	return result;
}

//-----------------------------------------------

bool draw_mesh(const arma::fvec& color, const arma::mat& vertices,
			   const arma::fvec& position, const arma::fmat& rotation)
{
	model_t mesh = create_mesh(color, vertices, position, rotation);

	bool result = draw(mesh);

	destroy(mesh);

	return result;
}

//-----------------------------------------------

bool draw_gaussian(const arma::fvec& color, const arma::vec& mu, const arma::mat& sigma,
				   bool background, bool border)
{
	const int NB_POINTS = 60;

	arma::mat pts = get_gaussian_border_vertices(mu, sigma, NB_POINTS, true);

	arma::mat vertices(2, NB_POINTS * 3);

	if (background)
	{
		for (int i = 0; i < NB_POINTS - 1; ++i)
		{
			vertices(arma::span::all, i * 3) = mu(arma::span(0, 1));
			vertices(arma::span::all, i * 3 + 1) = pts(arma::span::all, i + 1);
			vertices(arma::span::all, i * 3 + 2) = pts(arma::span::all, i);
		}

		vertices(arma::span::all, (NB_POINTS - 1) * 3) = mu(arma::span(0, 1));
		vertices(arma::span::all, (NB_POINTS - 1) * 3 + 1) = pts(arma::span::all, 0);
		vertices(arma::span::all, (NB_POINTS - 1) * 3 + 2) = pts(arma::span::all, NB_POINTS - 1);
	}


	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	arma::fvec gaussian_color;
	if (color.n_rows == 4)
		gaussian_color = color;
	else
		gaussian_color = arma::fvec({color(0), color(1), color(2), 0.1f});

	bool result = false;

	if (background)
		result = draw_mesh(gaussian_color, vertices);

	glDisable(GL_BLEND);

	if (border) {
		arma::fvec darker_color = gaussian_color(arma::span(0, 2)) * 0.5f;
		result &= gfx2::draw_line(darker_color, pts);
	}

	return result;
}


/******************************** RAY CASTING ********************************/

ray_t create_ray(const arma::fvec& origin, int mouse_x, int mouse_y,
				 const arma::fmat& view, const arma::fmat& projection,
				 int window_width, int window_height)
{
	ray_t ray;

	ray.origin = origin;

	// Compute the ray in homogeneous clip coordinates (range [-1:1, -1:1, -1:1, -1:1])
	arma::fvec ray_clip(4);
	ray_clip(0) = (2.0f * mouse_x) / window_width - 1.0f;
	ray_clip(1) = 1.0f - (2.0f * mouse_y) / window_height;
	ray_clip(2) = -1.0f;
	ray_clip(3) = 1.0f;

	// Compute the ray in camera coordinates
	arma::fvec ray_eye = arma::inv(projection) * ray_clip;
	ray_eye(2) = -1.0f;
	ray_eye(3) = 0.0f;

	// Compute the ray in world coordinates
	arma::fvec ray_world = arma::inv(view) * ray_eye;
	ray.direction = arma::fvec(arma::normalise(ray_world)).rows(0, 2);

	return ray;
}

//-----------------------------------------------

bool intersects(const ray_t& ray, const arma::fvec& center, float radius,
				arma::fvec &result)
{
	arma::fvec O_C = ray.origin - center;
	float b = arma::dot(ray.direction, O_C);
	float c = arma::dot(O_C, O_C) - radius * radius;

	float det = b * b - c;

	if (det < 0.0f)
		return false;

	float t;

	if (det > 0.0f)
	{
		float t1 = -b + sqrtf(det);
		float t2 = -b - sqrtf(det);

		t = (t1 < t2 ? t1 : t2);
	}
	else
	{
		t = -b + sqrtf(det);
	}

	result = ray.origin + ray.direction * t;

	return true;
}


/********************************** OTHERS ***********************************/

arma::mat get_gaussian_background_vertices(const arma::vec& mu, const arma::mat& sigma,
									 	   int nb_points)
{
	arma::mat pts = get_gaussian_border_vertices(mu, sigma, nb_points, true);

	arma::mat vertices(2, nb_points * 3);

	// We need to ensure that the vertices will be in a counter-clockwise order
	arma::vec v1 = pts(arma::span::all, 0) - mu(arma::span(0, 1));
	arma::vec v2 = pts(arma::span::all, 1) - mu(arma::span(0, 1));

	if (atan2(v1(1), v1(0)) - atan2(v2(1), v2(0)) > 0.0) {
		for (int i = 0; i < nb_points - 1; ++i)
		{
			vertices(arma::span::all, i * 3) = mu(arma::span(0, 1));
			vertices(arma::span::all, i * 3 + 1) = pts(arma::span::all, i + 1);
			vertices(arma::span::all, i * 3 + 2) = pts(arma::span::all, i);
		}

		vertices(arma::span::all, (nb_points - 1) * 3) = mu(arma::span(0, 1));
		vertices(arma::span::all, (nb_points - 1) * 3 + 1) = pts(arma::span::all, 0);
		vertices(arma::span::all, (nb_points - 1) * 3 + 2) = pts(arma::span::all, nb_points - 1);
	} else {
		for (int i = 0; i < nb_points - 1; ++i)
		{
			vertices(arma::span::all, i * 3) = mu(arma::span(0, 1));
			vertices(arma::span::all, i * 3 + 1) = pts(arma::span::all, i);
			vertices(arma::span::all, i * 3 + 2) = pts(arma::span::all, i + 1);
		}

		vertices(arma::span::all, (nb_points - 1) * 3) = mu(arma::span(0, 1));
		vertices(arma::span::all, (nb_points - 1) * 3 + 1) = pts(arma::span::all, nb_points - 1);
		vertices(arma::span::all, (nb_points - 1) * 3 + 2) = pts(arma::span::all, 0);
	}

	return vertices;
}

//-----------------------------------------------

arma::mat get_gaussian_border_vertices(const arma::vec& mu, const arma::mat& sigma,
								 	   int nb_points, bool line_strip)
{
	arma::mat pts0 = arma::join_cols(arma::cos(arma::linspace<arma::rowvec>(0, 2 * arma::datum::pi, nb_points)),
									 arma::sin(arma::linspace<arma::rowvec>(0, 2 * arma::datum::pi, nb_points))
	);

	arma::vec eigval(2);
	arma::mat eigvec(2, 2);
	eig_sym(eigval, eigvec, sigma(arma::span(0, 1), arma::span(0, 1)));

	arma::mat R = eigvec * diagmat(sqrt(eigval));

	arma::mat pts = R * pts0 + arma::repmat(mu(arma::span(0, 1)), 1, nb_points);

	if (line_strip)
		return pts;

	arma::mat vertices(2, nb_points * 2);

	for (int i = 0; i < nb_points - 1; ++i)
	{
		vertices(arma::span::all, i * 2) = pts(arma::span::all, i);;
		vertices(arma::span::all, i * 2 + 1) = pts(arma::span::all, i + 1);
	}

	vertices(arma::span::all, (nb_points - 1) * 2) = pts(arma::span::all, 0);;
	vertices(arma::span::all, (nb_points - 1) * 2 + 1) = pts(arma::span::all, nb_points - 1);

	return vertices;
}

}
