#ifndef UTILITY_HPP
#define UTILITY_HPP

tf::Transform getTransformFromAngle(float angle) {

	tf::Quaternion q;
	q.setRPY(0, 0, -angle);
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0));
	transform.setRotation(q);

	return transform * tg;

}

#endif
