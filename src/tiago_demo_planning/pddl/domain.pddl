(define (domain tiago_demo)

    (:requirements :strips :typing)

    (:types
        waypoint robot ball
    )

    (:predicates
        (connected ?from ?to - waypoint)
        (robot-at ?v - robot ?wp - waypoint)
        (ball-at ?b - ball ?wp - waypoint)
        (in-hand ?b - ball ?v - robot)
    )

    ;; Move between any two waypoints, avoiding terrain
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

        )
        :effect (and
            (not (ball-at ?b ?wp))
            (in-hand ?b ?v)
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
        )
    )

    (:action locate
        :parameters (?v - robot ?b - ball ?wp - waypoint)
        :precondition (and 
            (robot-at ?v ?wp)
            (no (ball-at ?b ?wp))
        )
        :effect (and
            (K (ball-at ?b ?wp))
        ) 
    )

)