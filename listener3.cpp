#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <fstream>
#include <sstream>

#include <cmath>

#define mp make_pair
#define pb push_back

using namespace std;
using namespace pcl;
using namespace sensor_msgs;
using namespace cv;

/*
top = 24 last 0
bot = 463 first 0

25 -> 0
462 -> 437

left = 16 last 0
right = 593 first 0

17 -> 0
592 -> 575

*/
const int HEIGHT = 438, WIDTH = 576;

Point3d xyz[HEIGHT][WIDTH];
Mat src(HEIGHT, WIDTH, CV_8UC3);
Mat pic(HEIGHT, WIDTH, CV_8UC3);
bool hasRead = false;
vector<Point> corners;
// ofstream fout("test.out");

bool cmp(const Point& p1, const Point& p2) {
	return p1.y == p2.y ? p1.x < p2.x : p1.y < p2.y;
}

void call_back(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
	pcl::fromROSMsg(*cloud, PointCloudXYZRGB);

	int max_i = cloud->height * cloud->width;
	for (int i = 0; i < max_i; i++) {
		int r = i / cloud->width, c = i % cloud->width;
		r -= 25; c -= 17;
		if (!(0 <= r && r < HEIGHT && 0 <= c && c < WIDTH)) {
			continue;
		}
		PointXYZRGB p = PointCloudXYZRGB.points[i];
		float z = p.z;
		xyz[r][c] = Point3d(p.x, -p.y, p.z);
		src.at<Vec3b>(Point(c, r)) = Vec3b(p.b, p.g, p.r);
		pic.at<Vec3b>(Point(c, r)) = Vec3b((int) (z * 100), (int) (z * 100), (int) (z * 100));
	}
	hasRead = true;
}

void read() {
	hasRead = false;
	while (!hasRead) {
		ros::spinOnce();
	}
}

void show() {
	// for (int i = 0; i < corners.size(); i++) {
	// 	// cout << '(' << corners[i].first << ", " << corners[i].second << ')' << endl;
	// 	circle(src, Point(corners[i].x, corners[i].y), 5, Scalar(0, 0, 255), -1);
	// }
	imshow("show1", src);
	if (waitKey(700) != -1) exit(0);
	// imshow("show1", pic);
	// cout << xyz[70][70].z << endl;
	// int ct = 0;
	// for (int i = 0; i < HEIGHT; i += 100) {
	// 	for (int j = 0; j < WIDTH; j += 100) {
	// 		float z = xyz[i][j].z;
	// 		// if (!(z != z)) ct++;
	// 		cout << 1/xyz[i][j].z << ' ';
	// 		// cout << '|' << z << ' ' << (z==z);
	// 	}
	// 	cout << endl;
	// }
	// cout << ct << endl;
}

void compress_lines(vector<Vec2f>& lines) {
	vector<double> weights(lines.size(), 1);
	for (int i = 0; i < lines.size(); i++) {
		double total_r = lines[i][0], total_a = lines[i][1];
		for (int j = lines.size() - 1; j > i; j--) {
			double diff_a = abs(lines[i][1] - lines[j][1]);
			if (abs(lines[i][0] - lines[j][0]) < 50) {
				if (diff_a < .2) {
					total_r += lines[j][0];
					total_a += lines[j][1];
					weights[i] += weights[j];
					lines.erase(lines.begin() + j);
				} else if (diff_a > CV_PI - .2) {
					total_r += lines[j][0];
					total_a += lines[j][1] > CV_PI / 2 ? lines[j][1] - CV_PI : lines[j][1];
					weights[i] += weights[j];
					lines.erase(lines.begin() + j);
				}
			}
		}

		lines[i][0] = total_r / weights[i];
		lines[i][1] = total_a / weights[i];
		if (lines[i][1] < 0) lines[i][1] += CV_PI;
	}
}

vector<Point> inter(const vector<Vec2f>& lines) {
	vector<Point> points;
	for (int i = 0; i < lines.size(); i++) {
		double r1 = lines[i][0], a1 = lines[i][1];
		// cout << r1 << ' ' << a1 << endl;
		for (int j = i + 1; j < lines.size(); j++) {
			double r2 = lines[j][0], a2 = lines[j][1];
			double x = (r1 / sin(a1) - r2 / sin(a2)) / (cos(a1) / sin(a1) - cos(a2) / sin(a2));
			double y = (-cos(a1) / sin(a1)) * x + r1 / sin(a1);
			if (-1 < x && x < WIDTH && -1 < y && y < HEIGHT && abs(a1 - a2) > .2 && abs(a1 - a2) < CV_PI - .2) {
				points.pb(Point(x, y));

			}
		}
	}
	return points;
}

Point3d trans(const Point3d& p, double h, double a, double d) {
	double x1 = p.x, y1 = p.y, z1 = p.z;
	double x2 = x1, y2 = y1*sin(a) - z1*cos(a) + h, z2 = y1*cos(a) + z1*sin(a);
	double x3 = d - z2, y3 = x2, z3 = y2;
	return Point3d(x3, y3, z3);
}

vector<Point> compress_points(const vector<Point>& points) {
	vector<pair<pair<double, double>, int>> cluster;
	for (int i = 0; i < points.size(); i++) {
		cluster.pb(mp(mp(points[i].x, points[i].y), 1));
	}
	while (cluster.size() > 4) {
		int i1 = 0, i2 = 1;
		pair<double, double> c1 = cluster[0].first, c2 = cluster[1].first;
		double dist = hypot(c1.first - c2.first, c1.second - c2.second);
		for (int i = 0; i < cluster.size(); i++) {
			pair<double, double> c3 = cluster[i].first;
			for (int j = i + 1; j < cluster.size(); j++) {
				pair<double, double> c4 = cluster[j].first;
				double dist2 = hypot(c3.first - c4.first, c3.second - c4.second);
				if (dist2 < dist) {
					c1 = c3;
					c2 = c4;
					dist = dist2;
					i1 = i;
					i2 = j;
				}
			}
		}

		double xsum1 = cluster[i1].first.first * cluster[i1].second;
		double ysum1 = cluster[i1].first.second * cluster[i1].second;
		double xsum2 = cluster[i2].first.first * cluster[i2].second;
		double ysum2 = cluster[i2].first.second * cluster[i2].second;
		int total = cluster[i1].second + cluster[i2].second;
		cluster.erase(cluster.begin() + i2);
		cluster.erase(cluster.begin() + i1);
		cluster.pb(mp(mp((xsum1 + xsum2) / total, (ysum1 + ysum2) / total), total));
	}

	vector<Point> points2;
	for (int i = 0; i < cluster.size(); i++) {
		points2.pb(Point(cvRound(cluster[i].first.first), cluster[i].first.second));
	}
	return points2;
}

bool get_corners() {
	Mat dst, cdst, src2;
	GaussianBlur(src, src2, Size(3, 3), 0, 0);
	Canny(src, dst, 80, 240, 3);
	cvtColor(dst, cdst, CV_GRAY2BGR);

	// cout << "d1" << endl;

	vector<Vec2f> lines;
	HoughLines(dst, lines, 3, CV_PI/180.0, 150, 0, 0 );
	// cout << "lines " << lines.size() << endl;
	compress_lines(lines);
	// cout << "lines " << lines.size() << endl;
	imshow("show1", cdst);
	if (waitKey(700) != -1) exit(0);
	for (int i = 0; i < lines.size(); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line(cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
	}

	if (lines.size() > 100) return false;

	/*
	23
	14
	



	*/


	vector<Point> points = inter(lines);
	corners = compress_points(points);
	sort(corners.begin(), corners.end(), cmp);

	// for (int i = 0; i < corners.size(); i++) {
	// 	Point p = corners[i];
	// 	cout << p.x << ' ' << p.y << endl;
	// 	cout << xyz[p.y][p.x].x << ' ' << xyz[p.y][p.x].y << ' ' << xyz[p.y][p.x].z << endl;
	// }
	// cout << "----------------------------" << endl;

	// imshow("show1", dst);
	// if (waitKey(700) != -1) exit(0);
	imshow("show1", cdst);
	if (waitKey(700) != -1) exit(0);

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, call_back);

	namedWindow("show1", CV_WINDOW_AUTOSIZE);

	// publish corners
	ros::init(argc, argv, "talker2");
	ros::NodeHandle m;
	ros::Publisher pub = m.advertise<std_msgs::String>("corners", 1000);
	ros::Rate loop_rate(10);

	// int ct = 0;
	// while (ct++ < 2) {
	// 	do {
	// 		read();
	// 	} while (!get_corners());
	// }
	// int r1 = -1, c1 = -1, r2 = -1, c2 = -1;
	// double dist1 = 1<<27, dist2 = 1<<27;
	// for (int i = 0; i < HEIGHT; i++) {
	// 	for (int j = 0; j < WIDTH; j++) {
	// 		// cout << i << ' ' << j << ' ' << xyz[i][j].z << (xyz[i][j].z * xyz[i][j].z > 0) << endl;
	// 		if (hypot(xyz[i][j].x, xyz[i][j].y) < dist1) {
	// 			dist1 = hypot(xyz[i][j].x, xyz[i][j].y);
	// 			r1 = i; c1 = j;
	// 		}
	// 		if (hypot(xyz[i][j].x, xyz[i][j].y - .1) < dist2) {
	// 			dist2 = hypot(xyz[i][j].x, xyz[i][j].y - .1);
	// 			r2 = i; c2 = j;
	// 		}
	// 	}
	// }
	// double gap = .1;
	// double z1 = xyz[r1][c1].z, z2 = xyz[r2][c2].z;
	// cout << xyz[r1][c1].x << ' ' << xyz[r1][c1].y << ' ' << xyz[r2][c2].x << ' ' << xyz[r2][c2].y << endl;
	// cout << "z1 " << z1 << endl;
	// cout << "z2 " << z2 << endl;
	// double theta2 = atan(gap / z1);
	// double theta = atan((z2 * cos(theta2) - z1) / (z2 * sin(theta2)));
	// double theta = atan2(z2 * cos(theta2) - z1, z2 * sin(theta2));
	// double h = z1 * cos(theta);
	// cout << "h1 " << h << endl;
	// cout << "theta1 " << theta * 180 / CV_PI << endl;
	// double c = z2/(sqrt(z1*z1+gap*gap) - 1) * z1*sqrt(z1*z1+gap*gap) / gap;
	// h = z1 / sqrt(c*c + 1);
	// theta = acos(h/z1);
	// cout << "theta2 " << theta * 180 / CV_PI << endl;
	// cout << "h2 " << h << endl;
	// cout << z1 << endl;
	// cout << z2 << endl;

	theta = 23.3 / 180 * CV_PI;
	h = .41;

	bool flag = false;
	while (true) {
		do {
			read();
		} while (!get_corners());
		show();
		if (corners.size() != 4) {
			continue;
		}
		for (int i = 0; i < corners.size(); i++) {
			Point3d pt = trans(xyz[corners[i].y][corners[i].x], h, theta, .502);
			if (!(pt.z * pt.z > 0)) flag = true;
		}
		if (flag) {
			flag = false;
			continue;
		}

		std_msgs::String msg;
		std::stringstream ss;
		Point3d pt = trans(xyz[corners[0].y][corners[0].x], h, theta, .502); // fill
		ss << pt.x << ' ' << pt.y << ' ' << pt.z << endl;
		pt = trans(xyz[corners[1].y][corners[1].x], h, theta, .502); // fill
		ss << pt.x << ' ' << pt.y << ' ' << pt.z << endl;
		// pt = trans(xyz[corners[2].y][corners[2].x], h, theta, .502); // fill
		// ss << ' ' << pt.x << ' ' << pt.y << ' ' << pt.z << endl;
		// pt = trans(xyz[corners[3].y][corners[3].x], h, theta, .502); // fill
		// ss << ' ' << pt.x << ' ' << pt.y << ' ' << pt.z << endl << "--------------------------------------";
		
		/*
		ss << endl;
		pt = trans(xyz[corners[0].y][corners[0].x], h, theta, .502);
		ss << pt.x - 0.17 << ' ' << pt.y << ' ' << pt.z << endl;
		pt = trans(xyz[corners[1].y][corners[1].x], h, theta, .502); // fill
		ss << ' ' << pt.x - 0.17 << ' ' << pt.y << ' ' << pt.z << endl;
		pt = trans(xyz[corners[2].y][corners[2].x], h, theta, .502); // fill
		ss << ' ' << pt.x - 0.17 << ' ' << pt.y << ' ' << pt.z << endl;
		pt = trans(xyz[corners[3].y][corners[3].x], h, theta, .502); // fill
		ss << ' ' << pt.x - 0.17 << ' ' << pt.y << ' ' << pt.z << endl << "--------------------------------------";
		ss << "--------------------------------" << endl;
		*/


		msg.data = ss.str();
		pub.publish(msg);
		cout << msg.data << endl;
		loop_rate.sleep();
		// Point3d pt = xyz[corners[0].y][corners[0].x];
		// std_msgs::String msg;
		// std::stringstream ss;
		// ss << pt.x << ' ' << pt.y << ' ' << pt.z;
		// pt = xyz[corners[1].y][corners[1].x];
		// ss << ' ' << pt.x << ' ' << pt.y << ' ' << pt.z;
		// msg.data = ss.str();
		// pub.publish(msg);
		// loop_rate.sleep();

	}

	return 0;
}