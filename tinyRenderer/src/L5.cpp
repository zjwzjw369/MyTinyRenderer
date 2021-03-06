#include "../include/tgaimage.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "../include/tiny_obj_loader.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include "../include/geometry.h"
#include "../include/model.h"
#include "../include/shader_gl.h"

#define SCREEN_WIDTH 500
#define SCREEN_HEIGHT 500
#define SCREEN_DEPTH 255
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
float zbuffer[SCREEN_WIDTH * SCREEN_HEIGHT];
vec3 eye(1, 1, 3);
vec3 center(0, 0, 0);
Model *model = NULL;

struct CustomShader : public IShader {
	struct UniformValue {
		mat4 Viewport, Projection, ModelView;
		
	} uniform_value;

	TGAColor sampler_tex0(vec4 uv) {
		return model->diffuse(embed<2>(uv));
	}

	struct VSOUT
	{
		vec4 gl_vertex;
		vec4 uv0;
	} vs_out[3];

	struct PSIN
	{
		vec4 gl_vertex;
		vec4 uv0;
	} ps_in;


	virtual void vs_out_to_ps_in(vec3 bar, bool interpolate = true) {
		if (!interpolate)
			return;
		ps_in.gl_vertex = vec4();
		ps_in.uv0 = vec4();
		for (int i = 0; i < 3; ++i) {
			ps_in.gl_vertex = vs_out[i].gl_vertex * bar[i] + ps_in.gl_vertex;
			ps_in.uv0 = vs_out[i].uv0 * bar[i] + ps_in.uv0;
		}
	}

	virtual vec4 vertex(int nthvert) {
		vs_out[nthvert].gl_vertex = uniform_value.Viewport * uniform_value.Projection * uniform_value.ModelView * vs_in.gl_vertex;
		vs_out[nthvert].uv0 =  vs_in.uv0;
		return vs_out[nthvert].gl_vertex;
	}

	virtual bool fragment(TGAColor &color) {
		color = this->sampler_tex0(ps_in.uv0);
		return false;                              // no, we do not discard this pixel
	}
};

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
	// 改进的Bresenham算法
	bool steep = false;
	if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
		std::swap(x0, y0);
		std::swap(x1, y1);
		steep = true;
	}
	if (x0 > x1) {
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = y1 - y0;
	int derror2 = std::abs(dy) * 2;
	int error2 = 0;
	int y = y0;
	for (int x = x0; x <= x1; x++) {
		if (steep) {
			image.set(y, x, color);
		}
		else {
			image.set(x, y, color);
		}
		error2 += derror2;
		if (error2 > dx) {
			y += (y1 > y0 ? 1 : -1);
			error2 -= dx * 2;
		}
	}
}


void triangle(vec2 t0, vec2 t1, vec2 t2, TGAImage &image, TGAColor color) {
	if (t0.y == t1.y && t0.y == t2.y) return; // I dont care about degenerate triangles 
	// sort the vertices, t0, t1, t2 lower−to−upper (bubblesort yay!) 
	if (t0.y > t1.y) std::swap(t0, t1);
	if (t0.y > t2.y) std::swap(t0, t2);
	if (t1.y > t2.y) std::swap(t1, t2);
	int total_height = t2.y - t0.y;
	for (int i = 0; i < total_height; i++) {
		bool second_half = i > t1.y - t0.y || t1.y == t0.y;
		int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
		float alpha = (float)i / total_height;
		float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height; // be careful: with above conditions no division by zero here 
		vec2 A = t0 + (t2 - t0)*alpha;
		vec2 B = second_half ? t1 + (t2 - t1)*beta : t0 + (t1 - t0)*beta;
		if (A.x > B.x) std::swap(A, B);
		for (int j = A.x; j <= B.x; j++) {
			image.set(j, t0.y + i, color); // attention, due to int casts t0.y+i != A.y 
		}
	}
}

vec3 barycentric(vec3 *pts, vec3 P) {
	vec3 u = cross(vec3(pts[2][0] - pts[0][0], pts[1][0] - pts[0][0], pts[0][0] - P[0]), vec3(pts[2][1] - pts[0][1], pts[1][1] - pts[0][1], pts[0][1] - P[1]));
	/* `pts` and `P` has integer value as coordinates
	   so `abs(u[2])` < 1 means `u[2]` is 0, that means
	   triangle is degenerate, in this case return something with negative coordinates */
	if (std::abs(u[2]) < 1) return vec3(-1, 1, 1);
	return vec3(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
}

void triangle_barycentric(vec3 *pts, TGAImage &image, CustomShader &shader) {
	vec2 bbox_min(image.get_width() - 1, image.get_height() - 1);
	vec2 bbox_max(0, 0);
	vec2 clamp(image.get_width() - 1, image.get_height() - 1);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			bbox_min[j] = std::max(0.0, std::min(bbox_min[j], pts[i][j]));
			bbox_max[j] = std::min(clamp[j], std::max(bbox_max[j], pts[i][j]));
		}
	}
	vec3 P;
	TGAColor color;
	for (P.x = (int)bbox_min.x; P.x <= (int)bbox_max.x; P.x++) {
		for (P.y = (int)bbox_min.y; P.y <= (int)bbox_max.y; P.y++) {
			vec3 bc_screen = barycentric(pts, P);
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
			P.z = 0;
			for (int i = 0; i < 3; i++)
			{
				P.z += pts[i][2] * bc_screen[i];
			}
			shader.vs_out_to_ps_in(bc_screen); // 模拟一下vs_out->ps_in的插值
			if (zbuffer[int(P.x + P.y * SCREEN_WIDTH)] < P.z) {
				if (!shader.fragment(color)) { // 模拟discard
					image.set(P.x, P.y, color);
				}
				zbuffer[int(P.x + P.y * SCREEN_WIDTH)] = P.z;
			}
		}
	}
}

mat4 lookAt(vec3 eye, vec3 center, vec3 up) {
	vec3 z = (eye - center).normalize();
	vec3 x = cross(up, z).normalize();
	vec3 y = cross(z, x).normalize();
	mat4 res = mat4::identity();
	for (size_t i = 0; i < 3; i++)
	{
		res[0][i] = x[i];
		res[1][i] = y[i];
		res[2][i] = z[i];
		res[i][3] = -center[i];
	}
	return res;
}

mat4 viewPort(int x, int y, int w, int h) {
	mat4 m = mat4::identity();
	m[0][3] = x + w / 2.0f;
	m[1][3] = y + h / 2.0f;
	m[2][3] = SCREEN_DEPTH / 2.0f;

	m[0][0] = w / 2.0f;
	m[1][1] = h / 2.0f;
	m[2][2] = SCREEN_DEPTH / 2.0f;
	return m;
}

void drawTriangle(Model* model, TGAImage &image, CustomShader &shader) {
	vec3 light_dir(0, 0, -1);
	for (int i = 0; i < model->nfaces(); i++) {		
		vec3 screen_space[3];
		for (int j = 0; j < 3; ++j) {
			shader.vs_in.gl_vertex = embed<4>(model->vert(i, j));
			shader.vs_in.uv0 = embed<4>(model->uv(i, j), 0.0);
			screen_space[j] = embed<3>(shader.vertex(j));
		}

		vec3 n = cross((screen_space[2] - screen_space[0]), (screen_space[1] - screen_space[0]));
		n.normalize();
		float intensity = n * light_dir;
		int col = int(intensity * 255);
		// 背面剔除
		if (intensity > 0) {
			vec3 pts[3] = { screen_space[0], screen_space[1], screen_space[2] };
			triangle_barycentric(pts, image, shader);
		}
	}
}

void clear_zbffer() {
	for (int i = 0; i < SCREEN_HEIGHT * SCREEN_WIDTH; ++i) {
		zbuffer[i] = -9999999999.0;
	}
}

void prepare_shader(CustomShader &shader, mat4 &model_view, mat4 &projection, mat4 &view_port) {
	shader.uniform_value.ModelView = model_view;
	shader.uniform_value.Projection = projection;
	shader.uniform_value.Viewport = view_port;
}

int main(int argc, char** argv) {
	clear_zbffer();
	TGAImage image(SCREEN_WIDTH, SCREEN_HEIGHT, TGAImage::RGB);
	model = new Model("obj/african_head/african_head.obj");
	mat4 model_view = lookAt(eye, center, vec3(0, 1, 0));
	mat4 projection = mat4::identity();
	projection[3][2] = -1.f / (eye - center).norm();
	mat4 view_port = viewPort(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	mat4 vpv_mat = view_port * projection * model_view;
	CustomShader shader;
	prepare_shader(shader, model_view, projection, view_port);
	drawTriangle(model, image, shader);
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	system("pause");
	return 0;
}