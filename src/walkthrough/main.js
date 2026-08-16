import * as THREE from 'three';
import { RectAreaLightUniformsLib } from 'three/examples/jsm/lights/RectAreaLightUniformsLib.js';
import { WebGLPathTracer } from 'three-gpu-pathtracer';

// ---------------------------------------------------------------------------
// Building layout
//
// Room A (x: -8..0)  - one wall left open to the outside ("picture window"),
//                       lit by a large sky-panel area light + sun panel.
// Doorway            - archway through the shared internal wall at x = 0.
// Room B (x: 0..8)   - fully enclosed, lit only by a warm ceiling lamp, so
//                       everything you see in there is bounced/indirect light.
// ---------------------------------------------------------------------------

const ROOM_HALF_Z = 3;
const WALL_HEIGHT = 3;
const WALL_T = 0.15;
const DOOR_HALF_W = 0.9;

const BUILDING_MIN_X = -7.6;
const BUILDING_MAX_X = 7.6;
const BUILDING_MIN_Z = -ROOM_HALF_Z + 0.4;
const BUILDING_MAX_Z = ROOM_HALF_Z - 0.4;

RectAreaLightUniformsLib.init();

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0x8fb8dd );

function addBox( w, h, d, x, y, z, color, roughness = 0.85, metalness = 0.0 ) {

	const mesh = new THREE.Mesh(
		new THREE.BoxGeometry( w, h, d ),
		new THREE.MeshPhysicalMaterial( { color, roughness, metalness } ),
	);
	mesh.position.set( x, y, z );
	scene.add( mesh );
	return mesh;

}

// floor / ceiling -------------------------------------------------------
addBox( 16, 0.2, ROOM_HALF_Z * 2, 0, -0.1, 0, 0x8a6a4c, 0.4 );
addBox( 8, 0.2, ROOM_HALF_Z * 2, -4, WALL_HEIGHT + 0.1, 0, 0xf2f2f2, 0.95 );
addBox( 8, 0.2, ROOM_HALF_Z * 2, 4, WALL_HEIGHT + 0.1, 0, 0xf2f2f2, 0.95 );

// Room A walls (north/south only - west side is open to the outside) ----
addBox( 8, WALL_HEIGHT, WALL_T, -4, WALL_HEIGHT / 2, ROOM_HALF_Z, 0xece4d8, 0.9 );
addBox( 8, WALL_HEIGHT, WALL_T, -4, WALL_HEIGHT / 2, -ROOM_HALF_Z, 0xece4d8, 0.9 );

// Room B walls (fully enclosed) -----------------------------------------
addBox( 8, WALL_HEIGHT, WALL_T, 4, WALL_HEIGHT / 2, ROOM_HALF_Z, 0xece4d8, 0.9 );
addBox( 8, WALL_HEIGHT, WALL_T, 4, WALL_HEIGHT / 2, -ROOM_HALF_Z, 0xece4d8, 0.9 );
addBox( WALL_T, WALL_HEIGHT, ROOM_HALF_Z * 2, 8, WALL_HEIGHT / 2, 0, 0xece4d8, 0.9 );

// Shared internal wall with a doorway archway ----------------------------
const doorSegW = ROOM_HALF_Z - DOOR_HALF_W;
addBox( WALL_T, WALL_HEIGHT, doorSegW, 0, WALL_HEIGHT / 2, ROOM_HALF_Z - doorSegW / 2, 0xdad2c4, 0.9 );
addBox( WALL_T, WALL_HEIGHT, doorSegW, 0, WALL_HEIGHT / 2, -( ROOM_HALF_Z - doorSegW / 2 ), 0xdad2c4, 0.9 );

// Outdoor ground strip beyond the open wall, for grounding the exterior --
addBox( 6, 0.2, ROOM_HALF_Z * 2 + 4, -11, -0.1, 0, 0x4c7a3f, 0.95 );

// Furniture ---------------------------------------------------------------
addBox( 1.6, 0.45, 0.7, -6, 0.225, 1.6, 0.85, 0.35, 0.0 );   // sofa seat
addBox( 1.6, 0.35, 0.15, -6, 0.575, 1.93, 0.85, 0.35, 0.0 ); // sofa back
addBox( 0.9, 0.35, 0.5, -3.6, 0.175, -1.6, 0xd8c9a3, 0.3 );  // coffee table
addBox( 1.6, 0.8, 0.6, 5.5, 0.4, -2.2, 0x3d3d3d, 0.2, 0.05 ); // desk
addBox( 1.4, 1.9, 0.5, 2.6, 0.95, 2.4, 0xb54a3a, 0.6 );       // wardrobe

const chromeBall = new THREE.Mesh(
	new THREE.SphereGeometry( 0.35, 32, 16 ),
	new THREE.MeshPhysicalMaterial( { color: 0xc0c0c0, roughness: 0.05, metalness: 1.0 } ),
);
chromeBall.position.set( 5.6, 0.35, 1.7 );
scene.add( chromeBall );

// Lighting ------------------------------------------------------------------

// Outdoor "sky" panel filling the open west wall of Room A - the main light
// source for that room, produces soft, physically bounced daylight + GI.
const sky = new THREE.RectAreaLight( 0xdfefff, 1400, ROOM_HALF_Z * 2, WALL_HEIGHT );
sky.position.set( -8, WALL_HEIGHT / 2, 0 );
sky.rotation.y = Math.PI / 2;
scene.add( sky );

// Low sun panel for a hint of directional warmth / harder shadows outside.
const sun = new THREE.RectAreaLight( 0xfff1d0, 4000, 3, 3 );
sun.position.set( -10, 2.2, 2 );
sun.rotation.y = Math.PI / 2 + 0.5;
sun.rotation.x = -0.3;
scene.add( sun );

// Warm ceiling lamp - the ONLY light source in Room B, so everything in
// that room is illuminated purely by ray-traced indirect bounce lighting.
const lamp = new THREE.RectAreaLight( 0xffd9a0, 900, 1.2, 1.2 );
lamp.position.set( 4, WALL_HEIGHT - 0.05, 0 );
lamp.rotation.x = Math.PI / 2;
scene.add( lamp );

// ---------------------------------------------------------------------------
// Renderer / path tracer
// ---------------------------------------------------------------------------

const canvas = document.getElementById( 'view' );
const renderer = new THREE.WebGLRenderer( { canvas, antialias: false } );
renderer.setPixelRatio( Math.min( window.devicePixelRatio, 2 ) );
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.3;

const camera = new THREE.PerspectiveCamera( 70, 1, 0.05, 100 );
camera.position.set( -5, 1.6, 0 );
let yaw = 0;
let pitch = 0;

const pathTracer = new WebGLPathTracer( renderer );
pathTracer.dynamicLowRes = true;     // instant rasterized preview while moving
pathTracer.lowResScale = 0.3;
pathTracer.renderScale = 0.85;       // full path-traced resolution once still
pathTracer.minSamples = 2;
pathTracer.fadeDuration = 400;
pathTracer.bounces = 6;
pathTracer.filterGlossyFactor = 0.5;
pathTracer.setScene( scene, camera );

function resize() {

	const w = window.innerWidth;
	const h = window.innerHeight;
	camera.aspect = w / h;
	camera.updateProjectionMatrix();
	renderer.setSize( w, h );
	pathTracer.updateCamera();

}
window.addEventListener( 'resize', resize );
resize();

// ---------------------------------------------------------------------------
// Movement / collision
// ---------------------------------------------------------------------------

function isBlocked( x, z ) {

	if ( x < BUILDING_MIN_X || x > BUILDING_MAX_X ) return true;
	if ( z < BUILDING_MIN_Z || z > BUILDING_MAX_Z ) return true;
	if ( Math.abs( x ) < WALL_T * 1.5 && Math.abs( z ) > DOOR_HALF_W ) return true;
	return false;

}

const MOVE_SPEED = 2.2; // m/s
const move = { x: 0, y: 0 }; // joystick vector: x = strafe, y = forward
let cameraDirty = true;

function applyLook( dx, dy ) {

	yaw -= dx * 0.0035;
	pitch -= dy * 0.0035;
	pitch = Math.max( -1.4, Math.min( 1.4, pitch ) );
	cameraDirty = true;

}

function updateMovement( dt ) {

	const mx = move.x, mz = -move.y;
	if ( mx !== 0 || mz !== 0 ) {

		const sin = Math.sin( yaw ), cos = Math.cos( yaw );
		const fx = ( mx * cos - mz * sin ) * MOVE_SPEED * dt;
		const fz = ( mx * sin + mz * cos ) * MOVE_SPEED * dt;

		const nx = camera.position.x + fx;
		const nz = camera.position.z + fz;
		if ( ! isBlocked( nx, camera.position.z ) ) camera.position.x = nx;
		if ( ! isBlocked( camera.position.x, nz ) ) camera.position.z = nz;
		cameraDirty = true;

	}

	camera.rotation.set( pitch, yaw, 0, 'YXZ' );

}

// --- keyboard (desktop) ---
const keys = new Set();
window.addEventListener( 'keydown', e => keys.add( e.code ) );
window.addEventListener( 'keyup', e => keys.delete( e.code ) );

function keyboardMove() {

	let x = 0, y = 0;
	if ( keys.has( 'KeyW' ) || keys.has( 'ArrowUp' ) ) y += 1;
	if ( keys.has( 'KeyS' ) || keys.has( 'ArrowDown' ) ) y -= 1;
	if ( keys.has( 'KeyA' ) || keys.has( 'ArrowLeft' ) ) x -= 1;
	if ( keys.has( 'KeyD' ) || keys.has( 'ArrowRight' ) ) x += 1;
	const len = Math.hypot( x, y );
	return len > 0 ? { x: x / len, y: y / len } : { x: 0, y: 0 };

}

// --- mouse drag look (desktop) ---
let mouseDown = false, lastMouseX = 0, lastMouseY = 0;
canvas.addEventListener( 'mousedown', e => {

	mouseDown = true; lastMouseX = e.clientX; lastMouseY = e.clientY;

} );
window.addEventListener( 'mouseup', () => mouseDown = false );
window.addEventListener( 'mousemove', e => {

	if ( ! mouseDown ) return;
	applyLook( e.clientX - lastMouseX, e.clientY - lastMouseY );
	lastMouseX = e.clientX; lastMouseY = e.clientY;

} );

// --- touch controls: left half = joystick, right half = look ---
const joystickBase = document.getElementById( 'joy-base' );
const joystickKnob = document.getElementById( 'joy-knob' );
let joyTouchId = null, joyOriginX = 0, joyOriginY = 0;
let lookTouchId = null, lookLastX = 0, lookLastY = 0;
const JOY_RADIUS = 55;

function onTouchStart( e ) {

	for ( const t of e.changedTouches ) {

		if ( t.clientX < window.innerWidth / 2 ) {

			if ( joyTouchId !== null ) continue;
			joyTouchId = t.identifier;
			joyOriginX = t.clientX;
			joyOriginY = t.clientY;
			joystickBase.style.display = 'block';
			joystickBase.style.left = `${ joyOriginX - 60 }px`;
			joystickBase.style.top = `${ joyOriginY - 60 }px`;
			joystickKnob.style.transform = 'translate(0px, 0px)';

		} else {

			if ( lookTouchId !== null ) continue;
			lookTouchId = t.identifier;
			lookLastX = t.clientX;
			lookLastY = t.clientY;

		}

	}

}

function onTouchMove( e ) {

	for ( const t of e.changedTouches ) {

		if ( t.identifier === joyTouchId ) {

			let dx = t.clientX - joyOriginX;
			let dy = t.clientY - joyOriginY;
			const d = Math.hypot( dx, dy );
			if ( d > JOY_RADIUS ) { dx = dx / d * JOY_RADIUS; dy = dy / d * JOY_RADIUS; }
			joystickKnob.style.transform = `translate(${ dx }px, ${ dy }px)`;
			move.x = dx / JOY_RADIUS;
			move.y = -dy / JOY_RADIUS;

		} else if ( t.identifier === lookTouchId ) {

			applyLook( t.clientX - lookLastX, t.clientY - lookLastY );
			lookLastX = t.clientX;
			lookLastY = t.clientY;

		}

	}

}

function onTouchEnd( e ) {

	for ( const t of e.changedTouches ) {

		if ( t.identifier === joyTouchId ) {

			joyTouchId = null;
			move.x = 0; move.y = 0;
			joystickBase.style.display = 'none';

		} else if ( t.identifier === lookTouchId ) {

			lookTouchId = null;

		}

	}

}

canvas.addEventListener( 'touchstart', e => { e.preventDefault(); onTouchStart( e ); }, { passive: false } );
canvas.addEventListener( 'touchmove', e => { e.preventDefault(); onTouchMove( e ); }, { passive: false } );
canvas.addEventListener( 'touchend', e => { e.preventDefault(); onTouchEnd( e ); }, { passive: false } );
canvas.addEventListener( 'touchcancel', e => { e.preventDefault(); onTouchEnd( e ); }, { passive: false } );

// ---------------------------------------------------------------------------
// Render loop
// ---------------------------------------------------------------------------

const clock = new THREE.Clock();
const statusEl = document.getElementById( 'status' );

function animate() {

	requestAnimationFrame( animate );

	const dt = Math.min( clock.getDelta(), 0.1 );
	const kb = keyboardMove();
	if ( kb.x !== 0 || kb.y !== 0 ) { move.x = kb.x; move.y = kb.y; }
	else if ( joyTouchId === null ) { move.x = 0; move.y = 0; }

	updateMovement( dt );

	if ( cameraDirty ) {

		pathTracer.updateCamera();
		cameraDirty = false;

	}

	pathTracer.renderSample();

	if ( statusEl ) {

		statusEl.textContent = `samples: ${ pathTracer.samples.toFixed( 1 ) }`;

	}

}

animate();
