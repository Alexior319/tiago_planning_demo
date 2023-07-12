(define (domain tiago_demo_2objs)
    (:requirements :strips :typing)
    (:types
        waypoint robot ball
    )
    (:predicates
        (connected ?from ?to - waypoint)
        (robot-at ?v - robot ?wp - waypoint)
        (ball-at ?b - ball ?wp - waypoint)
        (in-hand ?b - ball ?v - robot)
        (empty_hand ?v - robot)
    )
    (:action goto_waypoint
        :parameters (?v - robot ?from ?to - waypoint)
        :precondition (and
            (connected ?from ?to)
            (robot-at ?v ?from)
        )
        :effect (and
            (not (robot-at ?v ?from))
            (robot-at ?v ?to)
        )
    )
    (:action pick
        :parameters (?v - robot ?b - ball ?wp - waypoint)
        :precondition (and
            (ball-at ?b ?wp)
            (robot-at ?v ?wp)
            (empty_hand ?v)
        )
        :effect (and
            (not (ball-at ?b ?wp))
            (in-hand ?b ?v)
            (not (empty_hand ?v))
        )
    )
    (:action place
        :parameters (?v - robot ?b - ball ?wp - waypoint)
        :precondition (and
            (in-hand ?b ?v)
            (robot-at ?v ?wp)
        )
        :effect (and
            (not (in-hand ?b ?v))
            (ball-at ?b ?wp)
            (empty_hand ?v)
        )
    )
    (:action locate
        :parameters (?v - robot ?b - ball ?wp - waypoint)
        :precondition (and 
            (robot-at ?v ?wp)
            (UNKNOWN (ball-at ?b ?wp))
        )
        :effect (and
            (KNOWN (ball-at ?b ?wp))
        ) 
    )
    (:action locate_two_objs
        :parameters (?v - robot ?b ?b1 - ball ?wp - waypoint)
        :precondition (and 
            (robot-at ?v ?wp)
            (UNKNOWN (ball-at ?b ?wp))
            (UNKNOWN (ball-at ?b1 ?wp))
        )
        :effect (and
            (KNOWN (ball-at ?b ?wp))
            (KNOWN (ball-at ?b1 ?wp))
        ) 
    )
)