#include "../include/tgaimage.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "../include/tiny_obj_loader.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include "../include/geometry.h"
#include "../include/model.h"


#define SCREEN_WIDTH 500
#define SCREEN_HEIGHT 500

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
float zbuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

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

void triangle_barycentric(vec3 *pts, TGAImage &image, vec2 *uvs, Model *model) {
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
	vec2 uv;
	for (P.x = (int)bbox_min.x; P.x <= (int)bbox_max.x; P.x++) {
		for (P.y = (int)bbox_min.y; P.y <= (int)bbox_max.y; P.y++) {
			vec3 bc_screen = barycentric(pts, P);
			if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
			P.z = 0;
			uv.x = uv.y = 0;
			for (int i = 0; i < 3; i++) 
			{
				P.z += pts[i][2] * bc_screen[i];
				uv.x += uvs[i].x * bc_screen[i];
				uv.y += uvs[i].y * bc_screen[i];
			}
			if (zbuffer[int(P.x + P.y * SCREEN_WIDTH)] < P.z) {
				color = model->diffuse(uv);
				image.set(P.x, P.y, color);
				zbuffer[int(P.x + P.y * SCREEN_WIDTH)] = P.z;
			}

		}
	}
}

void drawTriangle(Model* model, TGAImage &image) {
	vec3 light_dir(0, 0, -1);
	for (int i = 0; i < model->nfaces(); i++) {
		vec3 v0 = model->vert(i, 0);
		vec3 v1 = model->vert(i, 1);
		vec3 v2 = model->vert(i, 2);

		vec2 uv0 = model->uv(i, 0);
		vec2 uv1 = model->uv(i, 1);
		vec2 uv2 = model->uv(i, 2);

		int x0 = (v0.x + 1) * SCREEN_WIDTH / 2.0 + 0.5;
		int y0 = (v0.y + 1.) * SCREEN_HEIGHT / 2.0 + 0.5;
		int x1 = (v1.x + 1) * SCREEN_WIDTH / 2.0 + 0.5;
		int y1 = (v1.y + 1.) * SCREEN_HEIGHT / 2.0 + 0.5;
		int x2 = (v2.x + 1) * SCREEN_WIDTH / 2.0 + 0.5;
		int y2 = (v2.y + 1.) * SCREEN_HEIGHT / 2.0 + 0.5;
		//	triangle(vec2(x0, y0),vec2(x1, y1), vec2(x2, y2), image, TGAColor(rand() * 255, rand() * 255, rand() * 255, 255));

		vec3 n = cross((v2 - v0), (v1 - v0));
		n.normalize();
		float intensity = n * light_dir;
		int col = int(intensity * 255);
		// 背面剔除
		if (intensity > 0) {
			vec3 pts[3] = { vec3(x0,y0, v0.z), vec3(x1, y1, v1.z), vec3(x2, y2, v2.z)};
			vec2 uvs[3] = { uv0, uv1, uv2};
			//triangle_barycentric(pts, image, TGAColor(col, col, col, 1), uvs);
			triangle_barycentric(pts, image, uvs, model);
		}
	}
}

void clear_zbffer() {
	for (int i = 0; i < SCREEN_HEIGHT * SCREEN_WIDTH; ++i) {
		zbuffer[i] = -9999999999.0;
	}
}

int main(int argc, char** argv) {
	clear_zbffer();
	TGAImage image(SCREEN_WIDTH, SCREEN_HEIGHT, TGAImage::RGB);
	Model model("obj/african_head/african_head.obj");
	drawTriangle(&model, image);
	image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
	image.write_tga_file("output.tga");
	system("pause");
	return 0;
}