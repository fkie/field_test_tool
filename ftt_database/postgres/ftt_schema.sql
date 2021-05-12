--
-- PostgreSQL database dump
--

-- Dumped from database version 10.5 (Ubuntu 10.5-0ubuntu0.18.04)
-- Dumped by pg_dump version 10.5 (Ubuntu 10.5-0ubuntu0.18.04)

-- Started on 2018-11-06 17:48:36 CET

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET client_min_messages = warning;
SET row_security = off;

--
-- TOC entry 6 (class 2615 OID 17884)
-- Name: topology; Type: SCHEMA; Schema: -; Owner: postgres
--

CREATE SCHEMA topology;


ALTER SCHEMA topology OWNER TO postgres;

--
-- TOC entry 4649 (class 0 OID 0)
-- Dependencies: 6
-- Name: SCHEMA topology; Type: COMMENT; Schema: -; Owner: postgres
--

COMMENT ON SCHEMA topology IS 'PostGIS Topology schema';


--
-- TOC entry 1 (class 3079 OID 13052)
-- Name: plpgsql; Type: EXTENSION; Schema: -; Owner: 
--

CREATE EXTENSION IF NOT EXISTS plpgsql WITH SCHEMA pg_catalog;


--
-- TOC entry 4650 (class 0 OID 0)
-- Dependencies: 1
-- Name: EXTENSION plpgsql; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION plpgsql IS 'PL/pgSQL procedural language';


--
-- TOC entry 2 (class 3079 OID 16385)
-- Name: postgis; Type: EXTENSION; Schema: -; Owner: 
--

CREATE EXTENSION IF NOT EXISTS postgis WITH SCHEMA public;


--
-- TOC entry 4651 (class 0 OID 0)
-- Dependencies: 2
-- Name: EXTENSION postgis; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION postgis IS 'PostGIS geometry, geography, and raster spatial types and functions';


--
-- TOC entry 3 (class 3079 OID 17885)
-- Name: postgis_topology; Type: EXTENSION; Schema: -; Owner: 
--

CREATE EXTENSION IF NOT EXISTS postgis_topology WITH SCHEMA topology;


--
-- TOC entry 4652 (class 0 OID 0)
-- Dependencies: 3
-- Name: EXTENSION postgis_topology; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION postgis_topology IS 'PostGIS topology spatial types and functions';


SET default_tablespace = '';

SET default_with_oids = false;

--
-- TOC entry 220 (class 1259 OID 18034)
-- Name: image; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.image (
    id integer NOT NULL,
    segment_id integer,
    image_filename character(512),
    image_data bytea,
    description character(512),
    secs double precision
);


ALTER TABLE public.image OWNER TO postgres;

--
-- TOC entry 221 (class 1259 OID 18040)
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
-- TOC entry 4653 (class 0 OID 0)
-- Dependencies: 221
-- Name: image_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.image_id_seq OWNED BY public.image.id;


--
-- TOC entry 222 (class 1259 OID 18042)
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
-- TOC entry 223 (class 1259 OID 18048)
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
-- TOC entry 4654 (class 0 OID 0)
-- Dependencies: 223
-- Name: ito_reason_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.ito_reason_id_seq OWNED BY public.ito_reason.id;


--
-- TOC entry 224 (class 1259 OID 18050)
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
    bag_file character(512),
    note text
);


ALTER TABLE public.leg OWNER TO postgres;

--
-- TOC entry 225 (class 1259 OID 18056)
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
-- TOC entry 4655 (class 0 OID 0)
-- Dependencies: 225
-- Name: leg_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.leg_id_seq OWNED BY public.leg.id;


--
-- TOC entry 226 (class 1259 OID 18058)
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
-- TOC entry 227 (class 1259 OID 18064)
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
-- TOC entry 4656 (class 0 OID 0)
-- Dependencies: 227
-- Name: note_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.note_id_seq OWNED BY public.note.id;


--
-- TOC entry 230 (class 1259 OID 18071)
-- Name: performer; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.performer (
    id integer NOT NULL,
    institution character(512)
);


ALTER TABLE public.performer OWNER TO postgres;

--
-- TOC entry 231 (class 1259 OID 18077)
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
-- TOC entry 4658 (class 0 OID 0)
-- Dependencies: 231
-- Name: performer_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.performer_id_seq OWNED BY public.performer.id;


--
-- TOC entry 232 (class 1259 OID 18079)
-- Name: personnel; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.personnel (
    id integer NOT NULL,
    name character varying(512),
    institution character varying(512)
);


ALTER TABLE public.personnel OWNER TO postgres;

--
-- TOC entry 233 (class 1259 OID 18085)
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
-- TOC entry 4659 (class 0 OID 0)
-- Dependencies: 233
-- Name: personnel_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.personnel_id_seq OWNED BY public.personnel.id;


--
-- TOC entry 234 (class 1259 OID 18087)
-- Name: pose; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.pose (
    id bigint NOT NULL,
    "position" public.geometry(Point,4326),
    secs double precision,
    segment_id integer,
    pose_source_id integer
);


ALTER TABLE public.pose OWNER TO postgres;

--
-- TOC entry 235 (class 1259 OID 18093)
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
-- TOC entry 4660 (class 0 OID 0)
-- Dependencies: 235
-- Name: pose_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.pose_id_seq OWNED BY public.pose.id;


--
-- TOC entry 236 (class 1259 OID 18095)
-- Name: pose_source; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.pose_source (
    id integer NOT NULL,
    key character(512),
    short_description character(512),
    long_description text
);


ALTER TABLE public.pose_source OWNER TO postgres;

--
-- TOC entry 237 (class 1259 OID 18101)
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
-- TOC entry 4661 (class 0 OID 0)
-- Dependencies: 237
-- Name: pose_source_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.pose_source_id_seq OWNED BY public.pose_source.id;


--
-- TOC entry 238 (class 1259 OID 18103)
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
    obstacle character(512),
    lighting character(512),
    slope character(512),
    distance double precision,
    start_position public.geometry(Point,4326)
);


ALTER TABLE public.segment OWNER TO postgres;

--
-- TOC entry 239 (class 1259 OID 18109)
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
-- TOC entry 4662 (class 0 OID 0)
-- Dependencies: 239
-- Name: segment_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.segment_id_seq OWNED BY public.segment.id;


--
-- TOC entry 240 (class 1259 OID 18111)
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
-- TOC entry 241 (class 1259 OID 18117)
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
-- TOC entry 4663 (class 0 OID 0)
-- Dependencies: 241
-- Name: segment_type_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.segment_type_id_seq OWNED BY public.segment_type.id;


--
-- TOC entry 242 (class 1259 OID 18119)
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
    test_intent character(512),
    workspace character(512),
    vehicle_id integer,
    note text,
    number integer
);


ALTER TABLE public.shift OWNER TO postgres;

--
-- TOC entry 243 (class 1259 OID 18125)
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
-- TOC entry 4664 (class 0 OID 0)
-- Dependencies: 243
-- Name: shift_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.shift_id_seq OWNED BY public.shift.id;


--
-- TOC entry 248 (class 1259 OID 18143)
-- Name: test_event; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.test_event (
    id integer NOT NULL,
    starttime_secs double precision,
    endtime_secs double precision,
    location character(512),
    version character(512),
    time_zone character(512),
    note text
);


ALTER TABLE public.test_event OWNER TO postgres;

--
-- TOC entry 249 (class 1259 OID 18149)
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
-- TOC entry 4667 (class 0 OID 0)
-- Dependencies: 249
-- Name: test_event_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.test_event_id_seq OWNED BY public.test_event.id;


--
-- TOC entry 250 (class 1259 OID 18151)
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
-- TOC entry 251 (class 1259 OID 18156)
-- Name: vehicle; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.vehicle (
    id integer NOT NULL,
    key character(512),
    short_description character(512),
    long_description text,
    institution character(512),
    configuration text
);


ALTER TABLE public.vehicle OWNER TO postgres;

--
-- TOC entry 252 (class 1259 OID 18162)
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
-- TOC entry 4668 (class 0 OID 0)
-- Dependencies: 252
-- Name: vehicle_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.vehicle_id_seq OWNED BY public.vehicle.id;


--
-- TOC entry 253 (class 1259 OID 18164)
-- Name: weather; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.weather (
    id integer NOT NULL,
    key character(512),
    short_description character(512),
    long_description text,
    icon_filename character varying(256)
);


ALTER TABLE public.weather OWNER TO postgres;

--
-- TOC entry 254 (class 1259 OID 18170)
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
-- TOC entry 4669 (class 0 OID 0)
-- Dependencies: 254
-- Name: weather_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.weather_id_seq OWNED BY public.weather.id;


--
-- TOC entry 4440 (class 2604 OID 18172)
-- Name: image id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.image ALTER COLUMN id SET DEFAULT nextval('public.image_id_seq'::regclass);


--
-- TOC entry 4441 (class 2604 OID 18173)
-- Name: ito_reason id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.ito_reason ALTER COLUMN id SET DEFAULT nextval('public.ito_reason_id_seq'::regclass);


--
-- TOC entry 4442 (class 2604 OID 18174)
-- Name: leg id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg ALTER COLUMN id SET DEFAULT nextval('public.leg_id_seq'::regclass);


--
-- TOC entry 4443 (class 2604 OID 18175)
-- Name: note id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note ALTER COLUMN id SET DEFAULT nextval('public.note_id_seq'::regclass);


--
-- TOC entry 4445 (class 2604 OID 18177)
-- Name: performer id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.performer ALTER COLUMN id SET DEFAULT nextval('public.performer_id_seq'::regclass);


--
-- TOC entry 4446 (class 2604 OID 18178)
-- Name: personnel id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.personnel ALTER COLUMN id SET DEFAULT nextval('public.personnel_id_seq'::regclass);


--
-- TOC entry 4447 (class 2604 OID 18179)
-- Name: pose id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose ALTER COLUMN id SET DEFAULT nextval('public.pose_id_seq'::regclass);


--
-- TOC entry 4448 (class 2604 OID 18180)
-- Name: pose_source id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose_source ALTER COLUMN id SET DEFAULT nextval('public.pose_source_id_seq'::regclass);


--
-- TOC entry 4449 (class 2604 OID 18181)
-- Name: segment id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment ALTER COLUMN id SET DEFAULT nextval('public.segment_id_seq'::regclass);


--
-- TOC entry 4450 (class 2604 OID 18182)
-- Name: segment_type id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment_type ALTER COLUMN id SET DEFAULT nextval('public.segment_type_id_seq'::regclass);


--
-- TOC entry 4451 (class 2604 OID 18183)
-- Name: shift id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift ALTER COLUMN id SET DEFAULT nextval('public.shift_id_seq'::regclass);


--
-- TOC entry 4454 (class 2604 OID 18186)
-- Name: test_event id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.test_event ALTER COLUMN id SET DEFAULT nextval('public.test_event_id_seq'::regclass);


--
-- TOC entry 4455 (class 2604 OID 18187)
-- Name: vehicle id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.vehicle ALTER COLUMN id SET DEFAULT nextval('public.vehicle_id_seq'::regclass);


--
-- TOC entry 4456 (class 2604 OID 18188)
-- Name: weather id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.weather ALTER COLUMN id SET DEFAULT nextval('public.weather_id_seq'::regclass);


--
-- TOC entry 4458 (class 2606 OID 18506)
-- Name: image image_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.image
    ADD CONSTRAINT image_pkey PRIMARY KEY (id);


--
-- TOC entry 4460 (class 2606 OID 18508)
-- Name: ito_reason ito_reason_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.ito_reason
    ADD CONSTRAINT ito_reason_pkey PRIMARY KEY (id);


--
-- TOC entry 4462 (class 2606 OID 18510)
-- Name: leg leg_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_pkey PRIMARY KEY (id);


--
-- TOC entry 4464 (class 2606 OID 18512)
-- Name: note note_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note
    ADD CONSTRAINT note_pkey PRIMARY KEY (id);


--
-- TOC entry 4468 (class 2606 OID 18516)
-- Name: performer performer_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.performer
    ADD CONSTRAINT performer_pkey PRIMARY KEY (id);


--
-- TOC entry 4470 (class 2606 OID 18518)
-- Name: personnel personnel_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.personnel
    ADD CONSTRAINT personnel_pkey PRIMARY KEY (id);


--
-- TOC entry 4472 (class 2606 OID 18520)
-- Name: pose pose_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose
    ADD CONSTRAINT pose_pkey PRIMARY KEY (id);


--
-- TOC entry 4474 (class 2606 OID 18522)
-- Name: pose_source pose_source_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose_source
    ADD CONSTRAINT pose_source_pkey PRIMARY KEY (id);


--
-- TOC entry 4477 (class 2606 OID 18524)
-- Name: segment segment_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_pkey PRIMARY KEY (id);


--
-- TOC entry 4479 (class 2606 OID 18526)
-- Name: segment_type segment_type_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment_type
    ADD CONSTRAINT segment_type_pkey PRIMARY KEY (id);


--
-- TOC entry 4481 (class 2606 OID 18528)
-- Name: shift shift_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_pkey PRIMARY KEY (id);


--
-- TOC entry 4487 (class 2606 OID 18534)
-- Name: test_event test_event_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.test_event
    ADD CONSTRAINT test_event_pkey PRIMARY KEY (id);


--
-- TOC entry 4489 (class 2606 OID 18536)
-- Name: vehicle vehicle_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.vehicle
    ADD CONSTRAINT vehicle_pkey PRIMARY KEY (id);


--
-- TOC entry 4491 (class 2606 OID 18538)
-- Name: weather weather_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.weather
    ADD CONSTRAINT weather_pkey PRIMARY KEY (id);


--
-- TOC entry 4475 (class 1259 OID 18539)
-- Name: segment_id; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX segment_id ON public.segment USING btree (id);


--
-- TOC entry 4492 (class 2606 OID 18540)
-- Name: image image_segment_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.image
    ADD CONSTRAINT image_segment_id_fkey FOREIGN KEY (segment_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- TOC entry 4493 (class 2606 OID 18545)
-- Name: leg leg_default_pose_source_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_default_pose_source_id_fkey FOREIGN KEY (default_pose_source_id) REFERENCES public.pose_source(id);


--
-- TOC entry 4494 (class 2606 OID 18550)
-- Name: leg leg_shift_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_shift_id_fkey FOREIGN KEY (shift_id) REFERENCES public.shift(id) ON DELETE CASCADE;


--
-- TOC entry 4495 (class 2606 OID 18555)
-- Name: leg leg_weather_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.leg
    ADD CONSTRAINT leg_weather_id_fkey FOREIGN KEY (weather_id) REFERENCES public.weather(id);


--
-- TOC entry 4496 (class 2606 OID 18560)
-- Name: note note_personnel_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note
    ADD CONSTRAINT note_personnel_id_fkey FOREIGN KEY (personnel_id) REFERENCES public.personnel(id);


--
-- TOC entry 4497 (class 2606 OID 18565)
-- Name: note note_segment_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.note
    ADD CONSTRAINT note_segment_id_fkey FOREIGN KEY (segment_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- TOC entry 4498 (class 2606 OID 18570)
-- Name: pose pose_pose_source_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose
    ADD CONSTRAINT pose_pose_source_id_fkey FOREIGN KEY (pose_source_id) REFERENCES public.pose_source(id);


--
-- TOC entry 4499 (class 2606 OID 18575)
-- Name: pose pose_segment_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.pose
    ADD CONSTRAINT pose_segment_id_fkey FOREIGN KEY (segment_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- TOC entry 4500 (class 2606 OID 18580)
-- Name: segment segment_ito_reason_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_ito_reason_id_fkey FOREIGN KEY (ito_reason_id) REFERENCES public.ito_reason(id);


--
-- TOC entry 4501 (class 2606 OID 18585)
-- Name: segment segment_leg_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_leg_id_fkey FOREIGN KEY (leg_id) REFERENCES public.leg(id) ON DELETE CASCADE;


--
-- TOC entry 4503 (class 2606 OID 18647)
-- Name: segment segment_parent_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_parent_id_fkey FOREIGN KEY (parent_id) REFERENCES public.segment(id) ON DELETE CASCADE;


--
-- TOC entry 4502 (class 2606 OID 18595)
-- Name: segment segment_segment_type_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.segment
    ADD CONSTRAINT segment_segment_type_id_fkey FOREIGN KEY (segment_type_id) REFERENCES public.segment_type(id);


--
-- TOC entry 4504 (class 2606 OID 18600)
-- Name: shift shift_performer_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_performer_id_fkey FOREIGN KEY (performer_id) REFERENCES public.performer(id);


--
-- TOC entry 4505 (class 2606 OID 18605)
-- Name: shift shift_robot_operator_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_robot_operator_id_fkey FOREIGN KEY (robot_operator_id) REFERENCES public.personnel(id);


--
-- TOC entry 4506 (class 2606 OID 18610)
-- Name: shift shift_safety_officer_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_safety_officer_id_fkey FOREIGN KEY (safety_officer_id) REFERENCES public.personnel(id);


--
-- TOC entry 4507 (class 2606 OID 18615)
-- Name: shift shift_test_administrator_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_test_administrator_id_fkey FOREIGN KEY (test_administrator_id) REFERENCES public.personnel(id);


--
-- TOC entry 4508 (class 2606 OID 18620)
-- Name: shift shift_test_director_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_test_director_id_fkey FOREIGN KEY (test_director_id) REFERENCES public.personnel(id);


--
-- TOC entry 4509 (class 2606 OID 18625)
-- Name: shift shift_test_event_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_test_event_id_fkey FOREIGN KEY (test_event_id) REFERENCES public.test_event(id);


--
-- TOC entry 4510 (class 2606 OID 18630)
-- Name: shift shift_vehicle_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.shift
    ADD CONSTRAINT shift_vehicle_id_fkey FOREIGN KEY (vehicle_id) REFERENCES public.vehicle(id);


-- Completed on 2018-11-06 17:48:36 CET

--
-- PostgreSQL database dump complete
--

