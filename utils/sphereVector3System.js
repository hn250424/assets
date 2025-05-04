function degreeToRadian(degree) {
    return degree * (Math.PI / 180)
}

function radianToDegree(radian) {
    return radian * (180 / Math.PI)
}

function getRadius(x, y, z) {
    return Math.sqrt(x ** 2 + y ** 2 + z ** 2)
}

function getTheta(x, y, z) {
    const radius = getRadius(x, y, z)
    return Math.acos(z / radius)
}

function getPhi(x, y) {
    return Math.atan2(y, x)
}

function getVector3(r, theta, phi) {
    const x = r * Math.sin(theta) * Math.cos(phi)
    const y = r * Math.sin(theta) * Math.sin(phi)
    const z = r * Math.cos(theta)
    return { x: x, y: y, z: z }
}

// *** Example.
const currentValue = {
    x: 0,
    y: 0,
    z: 9.6,
    theta: null,
    phi: null,
    radius: null
}

currentValue.radius = getRadius(currentValue.x, currentValue.y, currentValue.z)
currentValue.theta = radianToDegree(getTheta(currentValue.x, currentValue.y, currentValue.z))
currentValue.phi = radianToDegree(getPhi(currentValue.x, currentValue.y))

console.log('currentValue: ', currentValue)

const targetValue = {
    x: null,
    y: null,
    z: null,
    theta: 75,
    phi: 0,
    radius: currentValue.radius
}
const rTheta = degreeToRadian(targetValue.theta)
const rPhi = degreeToRadian(targetValue.phi)

const coordinates = getVector3(targetValue.radius, rTheta, rPhi)
targetValue.x = coordinates.x
targetValue.y = coordinates.y
targetValue.z = coordinates.z

console.log('targetValue: ', targetValue)
