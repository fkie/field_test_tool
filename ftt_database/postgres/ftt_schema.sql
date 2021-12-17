--
-- PostgreSQL database dump
--

-- Dumped from database version 10.16 (Ubuntu 10.16-0ubuntu0.18.04.1)
-- Dumped by pg_dump version 10.16 (Ubuntu 10.16-0ubuntu0.18.04.1)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

--
-- Name: topology; Type: SCHEMA; Schema: -; Owner: postgres
--

CREATE SCHEMA IF NOT EXISTS topology;


ALTER SCHEMA topology OWNER TO postgres;

--
-- Name: SCHEMA topology; Type: COMMENT; Schema: -; Owner: postgres
--

COMMENT ON SCHEMA topology IS 'PostGIS Topology schema';


--
-- Name: plpgsql; Type: EXTENSION; Schema: -; Owner: 
--

CREATE EXTENSION IF NOT EXISTS plpgsql WITH SCHEMA pg_catalog;


--
-- Name: EXTENSION plpgsql; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION plpgsql IS 'PL/pgSQL procedural language';


--
-- Name: postgis; Type: EXTENSION; Schema: -; Owner: 
--

CREATE EXTENSION IF NOT EXISTS postgis WITH SCHEMA public;


--
-- Name: EXTENSION postgis; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION postgis IS 'PostGIS geometry, geography, and raster spatial types and functions';


--
-- Name: postgis_topology; Type: EXTENSION; Schema: -; Owner: 
--

CREATE EXTENSION IF NOT EXISTS postgis_topology WITH SCHEMA topology;


--
-- Name: EXTENSION postgis_topology; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION postgis_topology IS 'PostGIS topology spatial types and functions';


SET default_tablespace = '';

SET default_with_oids = false;

--
-- Name: image; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.image (
    id integer NOT NULL,
    segment_id integer,
    image_filename character varying(512),
    image_data bytea,
    description character varying(512),
    secs double precision,
    orig_secs double precision
);


ALTER TABLE public.image OWNER TO postgres;

--
-- Name: image_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.image_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.image_id_seq OWNER TO postgres;

--
-- Name: image_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.image_id_seq OWNED BY public.image.id;


--
-- Name: ito_reason; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.ito_reason (
    id integer NOT NULL,
    key character varying(512),
    short_description character varying(512),
    long_description text,
    has_duration character(1),
    color character varying(32)
);


ALTER TABLE public.ito_reason OWNER TO postgres;

--
-- Name: ito_reason_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.ito_reason_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.ito_reason_id_seq OWNER TO postgres;

--
-- Name: ito_reason_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.ito_reason_id_seq OWNED BY public.ito_reason.id;


--
-- Name: leg; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.leg (
    id integer NOT NULL,
    shift_id integer,
    number integer,
    starttime_secs double precision,
    endtime_secs double precision,
    weather_id integer,
    default_pose_source_id integer,
    bag_file character varying(512),
    note text
);


ALTER TABLE public.leg OWNER TO postgres;

--
-- Name: leg_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.leg_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.leg_id_seq OWNER TO postgres;

--
-- Name: leg_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.leg_id_seq OWNED BY public.leg.id;


--
-- Name: local_pose; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.local_pose (
    id integer NOT NULL,
    segment_id integer NOT NULL,
    secs double precision,
    frame_id character varying(512),
    "position" public.geometry(Point),
    orig_secs double precision
);


ALTER TABLE public.local_pose OWNER TO postgres;

--
-- Name: local_pose_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.local_pose_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.local_pose_id_seq OWNER TO postgres;

--
-- Name: local_pose_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.local_pose_id_seq OWNED BY public.local_pose.id;


--
-- Name: map_image; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.map_image (
    id integer NOT NULL,
    shift_id integer NOT NULL,
    secs double precision,
    frame_id character varying(512),
    width integer,
    height integer,
    resolution double precision,
    origin public.geometry(Point),
    image_data bytea,
    orig_secs double precision
);


ALTER TABLE public.map_image OWNER TO postgres;

--
-- Name: map_image_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.map_image_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.map_image_id_seq OWNER TO postgres;

--
-- Name: map_image_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.map_image_id_seq OWNED BY public.map_image.id;


--
-- Name: note; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.note (
    id integer NOT NULL,
    segment_id integer,
    note text,
    secs double precision,
    personnel_id integer
);


ALTER TABLE public.note OWNER TO postgres;

--
-- Name: note_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.note_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.note_id_seq OWNER TO postgres;

--
-- Name: note_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.note_id_seq OWNED BY public.note.id;


--
-- Name: performer; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.performer (
    id integer NOT NULL,
    institution character varying(512)
);


ALTER TABLE public.performer OWNER TO postgres;

--
-- Name: performer_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.performer_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.performer_id_seq OWNER TO postgres;

--
-- Name: performer_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.performer_id_seq OWNED BY public.performer.id;


--
-- Name: personnel; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.personnel (
    id integer NOT NULL,
    name character varying(512),
    institution character varying(512)
);


ALTER TABLE public.personnel OWNER TO postgres;

--
-- Name: personnel_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.personnel_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.personnel_id_seq OWNER TO postgres;

--
-- Name: personnel_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.personnel_id_seq OWNED BY public.personnel.id;


--
-- Name: pose; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.pose (
    id bigint NOT NULL,
    "position" public.geometry(Point,4326),
    secs double precision,
    segment_id integer,
    pose_source_id integer,
    orig_secs double precision
);


ALTER TABLE public.pose OWNER TO postgres;

--
-- Name: pose_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.pose_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.pose_id_seq OWNER TO postgres;

--
-- Name: pose_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.pose_id_seq OWNED BY public.pose.id;


--
-- Name: pose_source; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.pose_source (
    id integer NOT NULL,
    key character varying(512),
    short_description character varying(512),
    long_description text
);


ALTER TABLE public.pose_source OWNER TO postgres;

--
-- Name: pose_source_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.pose_source_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.pose_source_id_seq OWNER TO postgres;

--
-- Name: pose_source_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.pose_source_id_seq OWNED BY public.pose_source.id;


--
-- Name: segment; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.segment (
    id integer NOT NULL,
    leg_id integer,
    parent_id integer,
    ito_reason_id integer,
    segment_type_id integer,
    starttime_secs double precision,
    endtime_secs double precision,
    obstacle character varying(512),
    lighting character varying(512),
    slope character varying(512),
    distance double precision,
    start_position public.geometry(Point,4326),
    local_start_position public.geometry(Point),
    orig_starttime_secs double precision
);


ALTER TABLE public.segment OWNER TO postgres;

--
-- Name: segment_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.segment_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.segment_id_seq OWNER TO postgres;

--
-- Name: segment_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.segment_id_seq OWNED BY public.segment.id;


--
-- Name: segment_type; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.segment_type (
    id integer NOT NULL,
    key character varying(512),
    short_description character varying(512),
    long_description text,
    color character varying(32)
);


ALTER TABLE public.segment_type OWNER TO postgres;

--
-- Name: segment_type_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.segment_type_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.segment_type_id_seq OWNER TO postgres;

--
-- Name: segment_type_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.segment_type_id_seq OWNED BY public.segment_type.id;


--
-- Name: shift; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.shift (
    id integer NOT NULL,
    test_event_id integer,
    starttime_secs double precision,
    endtime_secs double precision,
    test_administrator_id integer,
    test_director_id integer,
    safety_officer_id integer,
    robot_operator_id integer,
    performer_id integer,
    test_intent character varying(512),
    workspace character varying(512),
    vehicle_id integer,
    note text,
    number integer
);


ALTER TABLE public.shift OWNER TO postgres;

--
-- Name: shift_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.shift_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.shift_id_seq OWNER TO postgres;

--
-- Name: shift_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.shift_id_seq OWNED BY public.shift.id;


--
-- Name: test_event; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.test_event (
    id integer NOT NULL,
    starttime_secs double precision,
    endtime_secs double precision,
    location character varying(512),
    version character varying(512),
    time_zone character varying(512),
    note text
);


ALTER TABLE public.test_event OWNER TO postgres;

--
-- Name: test_event_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.test_event_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.test_event_id_seq OWNER TO postgres;

--
-- Name: test_event_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.test_event_id_seq OWNED BY public.test_event.id;


--
-- Name: v_pose_shift; Type: VIEW; Schema: public; Owner: postgres
--

CREATE VIEW public.v_pose_shift AS
 SELECT pose.id AS pose_id,
    segment.id AS segment_id,
    leg.id AS leg_id,
    leg.shift_id,
    segment.segment_type_id,
    pose."position" AS geom
   FROM ((public.pose
     JOIN public.segment ON ((pose.segment_id = segment.id)))
     JOIN public.leg ON ((segment.leg_id = leg.id)))
  WHERE (leg.shift_id = 10);


ALTER TABLE public.v_pose_shift OWNER TO postgres;

--
-- Name: vehicle; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.vehicle (
    id integer NOT NULL,
    key character varying(512),
    short_description character varying(512),
    long_description text,
    institution character varying(512),
    configuration text
);


ALTER TABLE public.vehicle OWNER TO postgres;

--
-- Name: vehicle_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.vehicle_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.vehicle_id_seq OWNER TO postgres;

--
-- Name: vehicle_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.vehicle_id_seq OWNED BY public.vehicle.id;


--
-- Name: weather; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.weather (
    id integer NOT NULL,
    key character varying(512),
    short_description character varying(512),
    long_description text,
    icon_filename character varying(256)
);


ALTER TABLE public.weather OWNER TO postgres;

--
-- Name: weather_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.weather_id_seq
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER TABLE public.weather_id_seq OWNER TO postgres;

--
-- Name: weather_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.weather_id_seq OWNED BY public.weather.id;


--
-- Name: image id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.image ALTER COLUMN id SET DEFAULT nextval('public.image_id_seq'::regclass);


--
-- Name: ito_reason id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.ito_reason ALTER COLUMN id SET DEFAULT nextval('public.ito_reason_id_seq'::regclass);


--
-- Name: leg id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg ALTER COLUMN id SET DEFAULT nextval('public.leg_id_seq'::regclass);


--
-- Name: local_pose id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.local_pose ALTER COLUMN id SET DEFAULT nextval('public.local_pose_id_seq'::regclass);


--
-- Name: map_image id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.map_image ALTER COLUMN id SET DEFAULT nextval('public.map_image_id_seq'::regclass);


--
-- Name: note id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note ALTER COLUMN id SET DEFAULT nextval('public.note_id_seq'::regclass);


--
-- Name: performer id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.performer ALTER COLUMN id SET DEFAULT nextval('public.performer_id_seq'::regclass);


--
-- Name: personnel id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.personnel ALTER COLUMN id SET DEFAULT nextval('public.personnel_id_seq'::regclass);


--
-- Name: pose id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose ALTER COLUMN id SET DEFAULT nextval('public.pose_id_seq'::regclass);


--
-- Name: pose_source id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose_source ALTER COLUMN id SET DEFAULT nextval('public.pose_source_id_seq'::regclass);


--
-- Name: segment id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment ALTER COLUMN id SET DEFAULT nextval('public.segment_id_seq'::regclass);


--
-- Name: segment_type id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment_type ALTER COLUMN id SET DEFAULT nextval('public.segment_type_id_seq'::regclass);


--
-- Name: shift id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift ALTER COLUMN id SET DEFAULT nextval('public.shift_id_seq'::regclass);


--
-- Name: test_event id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.test_event ALTER COLUMN id SET DEFAULT nextval('public.test_event_id_seq'::regclass);


--
-- Name: vehicle id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.vehicle ALTER COLUMN id SET DEFAULT nextval('public.vehicle_id_seq'::regclass);


--
-- Name: weather id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.weather ALTER COLUMN id SET DEFAULT nextval('public.weather_id_seq'::regclass);


--
-- Name: image image_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.image
    ADD CONSTRAINT image_pkey PRIMARY KEY (id);


--
-- Name: ito_reason ito_reason_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.ito_reason
    ADD CONSTRAINT ito_reason_pkey PRIMARY KEY (id);


--
-- Name: leg leg_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_pkey PRIMARY KEY (id);


--
-- Name: local_pose local_pose_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.local_pose
    ADD CONSTRAINT local_pose_pkey PRIMARY KEY (id);


--
-- Name: map_image map_image_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.map_image
    ADD CONSTRAINT map_image_pkey PRIMARY KEY (id);


--
-- Name: map_image map_image_shift_id_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.map_image
    ADD CONSTRAINT map_image_shift_id_key UNIQUE (shift_id);


--
-- Name: note note_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note
    ADD CONSTRAINT note_pkey PRIMARY KEY (id);


--
-- Name: performer performer_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.performer
    ADD CONSTRAINT performer_pkey PRIMARY KEY (id);


--
-- Name: personnel personnel_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.personnel
    ADD CONSTRAINT personnel_pkey PRIMARY KEY (id);


--
-- Name: pose pose_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose
    ADD CONSTRAINT pose_pkey PRIMARY KEY (id);


--
-- Name: pose_source pose_source_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose_source
    ADD CONSTRAINT pose_source_pkey PRIMARY KEY (id);


--
-- Name: segment segment_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_pkey PRIMARY KEY (id);


--
-- Name: segment_type segment_type_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment_type
    ADD CONSTRAINT segment_type_pkey PRIMARY KEY (id);


--
-- Name: shift shift_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_pkey PRIMARY KEY (id);


--
-- Name: test_event test_event_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.test_event
    ADD CONSTRAINT test_event_pkey PRIMARY KEY (id);


--
-- Name: vehicle vehicle_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.vehicle
    ADD CONSTRAINT vehicle_pkey PRIMARY KEY (id);


--
-- Name: weather weather_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.weather
    ADD CONSTRAINT weather_pkey PRIMARY KEY (id);


--
-- Name: segment_id; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX segment_id ON public.segment USING btree (id);


--
-- Name: image image_segment_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.image
    ADD CONSTRAINT image_segment_id_fkey FOREIGN KEY (segment_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- Name: leg leg_default_pose_source_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_default_pose_source_id_fkey FOREIGN KEY (default_pose_source_id) REFERENCES public.pose_source(id);


--
-- Name: leg leg_shift_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_shift_id_fkey FOREIGN KEY (shift_id) REFERENCES public.shift(id) ON DELETE CASCADE;


--
-- Name: leg leg_weather_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_weather_id_fkey FOREIGN KEY (weather_id) REFERENCES public.weather(id);


--
-- Name: local_pose local_pose_segment_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.local_pose
    ADD CONSTRAINT local_pose_segment_id_fkey FOREIGN KEY (segment_id) REFERENCES public.segment(id);


--
-- Name: map_image map_shift_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.map_image
    ADD CONSTRAINT map_shift_id_fkey FOREIGN KEY (shift_id) REFERENCES public.shift(id);


--
-- Name: note note_personnel_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note
    ADD CONSTRAINT note_personnel_id_fkey FOREIGN KEY (personnel_id) REFERENCES public.personnel(id);


--
-- Name: note note_segment_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note
    ADD CONSTRAINT note_segment_id_fkey FOREIGN KEY (segment_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- Name: pose pose_pose_source_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose
    ADD CONSTRAINT pose_pose_source_id_fkey FOREIGN KEY (pose_source_id) REFERENCES public.pose_source(id);


--
-- Name: pose pose_segment_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose
    ADD CONSTRAINT pose_segment_id_fkey FOREIGN KEY (segment_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- Name: segment segment_ito_reason_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_ito_reason_id_fkey FOREIGN KEY (ito_reason_id) REFERENCES public.ito_reason(id);


--
-- Name: segment segment_leg_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_leg_id_fkey FOREIGN KEY (leg_id) REFERENCES public.leg(id) ON DELETE CASCADE;


--
-- Name: segment segment_parent_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_parent_id_fkey FOREIGN KEY (parent_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- Name: segment segment_segment_type_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_segment_type_id_fkey FOREIGN KEY (segment_type_id) REFERENCES public.segment_type(id);


--
-- Name: shift shift_performer_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_performer_id_fkey FOREIGN KEY (performer_id) REFERENCES public.performer(id);


--
-- Name: shift shift_robot_operator_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_robot_operator_id_fkey FOREIGN KEY (robot_operator_id) REFERENCES public.personnel(id);


--
-- Name: shift shift_safety_officer_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_safety_officer_id_fkey FOREIGN KEY (safety_officer_id) REFERENCES public.personnel(id);


--
-- Name: shift shift_test_administrator_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_test_administrator_id_fkey FOREIGN KEY (test_administrator_id) REFERENCES public.personnel(id);


--
-- Name: shift shift_test_director_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_test_director_id_fkey FOREIGN KEY (test_director_id) REFERENCES public.personnel(id);


--
-- Name: shift shift_test_event_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_test_event_id_fkey FOREIGN KEY (test_event_id) REFERENCES public.test_event(id);


--
-- Name: shift shift_vehicle_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_vehicle_id_fkey FOREIGN KEY (vehicle_id) REFERENCES public.vehicle(id);


--
-- PostgreSQL database dump complete
--

