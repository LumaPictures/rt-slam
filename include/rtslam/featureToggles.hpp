/*
 * featureToggles.hpp
 *
 *  Created on: Jul 25, 2012
 *      Author: paulm
 */

#ifndef FEATURE_TOGGLES_HPP_
#define FEATURE_TOGGLES_HPP_

/** ############################################################################
 * #############################################################################
 * features enable/disable
 * ###########################################################################*/

/*
 * STATUS: working fine, use it
 * Ransac ensures that we use correct observations for a few first updates,
 * allowing bad observations to be more easily detected by gating
 * You can disable it by setting N_UPDATES_RANSAC to 0 in config file
 */
#define RANSAC_FIRST 1

/*
 * STATUS: working fine, use it
 * This allows to use Dala "atrv" robot model instead of camera (default) model in the Gdhe view display
 */
#define ATRV 0

/*
 * STATUS: working fine, use it
 * This allows to have 0% cpu used for waiting/idle
 */
//#define EVENT_BASED_RAW 1 // always enabled now

/*
 * STATUS: in progress, do not use for now
 * This allows to track landmarks longer by updating the reference patch when
 * the landmark is not detected anymore and the point of view has changed
 * significantly enough
 * The problem is that the correlation is not robust enough in a matching
 * (opposed to tracking) context, and it can provoke matching errors with a
 * progressive appearance drift.
 * Also decreasing perfs by 10%, probably because we save a view at each obs,
 * or maybe it just because of the different random process
 */
//#define MULTIVIEW_DESCRIPTOR 1 // moved in config file

/*
 * STATUS: in progress, do not use for now
 * This allows to ignore some landmarks when we have some experience telling
 * us that we can't observe this landmarks from here (masking), in order to
 * save some time, and to allow creation of other observations in the
 * neighborhood to keep localizing with enough landmarks.
 * The problem is that sometimes it creates too many landmarks in the same area
 * (significantly slowing down slam), and sometimes doesn't create enough of
 * them when it is necessary.
 */
#define VISIBILITY_MAP 0


/*
 * STATUS: in progress, do not use for now
 * Only update if expectation uncertainty is significant wrt measurement uncertainty.
 *
 * Large updates are causing inconsistency because of linearization errors,
 * but too numerous updates are also causing inconsistency,
 * so we should avoid to do not significant updates.
 * An update is not significant if there are large odds that it is
 * only measurement noise and that there is not much information.
 *
 * When the camera is not moving at all, the landmarks are converging anyway
 * quite fast because of this, at very unconsistent positions of course,
 * so that when the camera moves it cannot recover them.
 *
 * Some work needs to be done yet to prevent search ellipses from growing
 * too much and integrate it better with the whole management, but this was
 * for first evaluation purpose.
 *
 * Unfortunately it doesn't seem to improve much the situation, even if
 * it is still working correctly with less computations.
 * The feature is disabled for now.
 */
#define RELEVANCE_TEST 0

/*
 * STATUS: seems to improve things, needs more testing but you can try it
 * Only update P if expectation uncertainty is significant wrt measurement uncertainty.
 *
 * This is similar to RELEVANCE_TEST except that we always update mean, and
 * update covariance only if innovation is relevant wrt measurement noise,
 * and it is more stable than RELEVANCE_TEST.
 *
 * Needs testing to see if it is stable enough and how to tune the relevance
 * threshold.
 */
#define RELEVANCE_TEST_P 0

/*
 * STATUS: in progress
 * Ability to search for known image markers, to provide relative positioning
 *
 */

#define KNOWN_MARKER_SEARCH 1

/*
 * STATUS: in progress, do not use for now
 * This uses HDseg powered Segment based slam instead of the usual point based slam.
 * 0 use points
 * 1 use segments
 * 2 use both sgments and points
 */
#define SEGMENT_BASED 0
#if SEGMENT_BASED>1
	#define SEGMENT_NOISE_FACTOR 5
#else
	#define SEGMENT_NOISE_FACTOR 1
#endif

#if SEGMENT_BASED
	#ifndef HAVE_MODULE_DSEG
	#error "dseg module is required for segment based slam"
	#endif
#endif


/*
 * STATUS: seems to work ok, needs a bit more testing but you can try it
 * This option will allocate time to data managers to make them stop
 * updating observations when there is no time anymore, in order to avoid
 * missing frames.
 */

#define REAL_TIME_LIVE_RUN 0



#endif /* FEATURE_TOGGLES_HPP_ */
