{-# LANGUAGE FlexibleContexts      #-}
{-# LANGUAGE FlexibleInstances     #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-# LANGUAGE OverloadedStrings     #-}
{-# LANGUAGE QuasiQuotes           #-}
{-# LANGUAGE ScopedTypeVariables   #-}
{-# LANGUAGE Strict                #-}
{-# LANGUAGE TemplateHaskell       #-}
{-# LANGUAGE TypeFamilies          #-}
{-# LANGUAGE UndecidableInstances  #-}
{-# LANGUAGE ViewPatterns          #-}

module Apecs.Physics.Shape where

import           Apecs.Core
import           Control.Monad
import           Control.Monad.IO.Class (MonadIO, liftIO)
import           Data.Bits
import qualified Data.IntMap            as M
import qualified Data.IntSet            as S
import           Data.IORef
import           Data.Monoid            ((<>))
import qualified Data.Vector.Storable   as V
import qualified Data.Vector.Unboxed    as U
import           Foreign.ForeignPtr
import           Foreign.Ptr
import qualified Language.C.Inline      as C
-- import           Linear.V2

import           Apecs.Physics.Space    ()
import           Apecs.Physics.Types
import           Foreign                (Storable (peek), with)
import           Geomancy.Vec2          (Vec2 (..), toTuple)
import           Text.Printf            (printf)

C.context (phycsCtx <> C.vecCtx)
C.include "<chipmunk.h>"

maskAll, maskNone :: Bitmask
maskAll  = complement zeroBits
maskNone = zeroBits
-- | Makes a bitmask from a list of indices
maskList :: [Int] -> Bitmask
maskList = foldr (flip setBit) maskNone

defaultFilter :: CollisionFilter
defaultFilter = CollisionFilter 0 maskAll maskAll

-- | A box with the given height, width, and center point
boxShape :: Float -> Float -> Vec -> Convex
boxShape w h offset = Convex ((+offset) <$> verts) 0
  where
    w' = w/2
    h' = h/2
    verts = [ Vec2 (-w') (-h')
            , Vec2 (-w') h'
            , Vec2 w' h'
            , Vec2 w' (-h') ]

instance Component Shape where
  type Storage Shape = Space Shape

instance (MonadIO m, Has w m Physics) => Has w m Shape where
  getStore = (cast :: Space Physics -> Space Shape) <$> getStore

instance MonadIO m => ExplMembers m (Space Shape) where
  explMembers (Space _ sMap _ _ _) = liftIO $ U.fromList . M.keys <$> readIORef sMap

instance MonadIO m => ExplDestroy m (Space Shape) where
  explDestroy (Space bMap sMap _ _ spc) sEty = liftIO $ do
    rd <- M.lookup sEty <$> readIORef sMap
    forM_ rd $ \(Record sPtr (Shape (Entity bEty) _)) -> do
      rd <- M.lookup bEty <$> readIORef bMap
      forM_ rd $ \bRec -> modifyIORef' (brShapes bRec) (S.delete sEty)
      modifyIORef' sMap (M.delete sEty)
      destroyShape spc sPtr

instance MonadIO m => ExplSet m (Space Shape) where
  explSet sp@(Space bMap sMap _ _ spcPtr) sEty shape@(Shape (Entity bEty) sh) = liftIO $ do
    explDestroy sp sEty
    rd <- M.lookup bEty <$> readIORef bMap
    forM_ rd $ \bRec -> do
      shPtr <- newShape spcPtr (brPtr bRec) sh sEty
      modifyIORef' (brShapes bRec) (S.insert sEty)
      modifyIORef' sMap (M.insert sEty (Record shPtr shape))

instance MonadIO m => ExplGet m (Space Shape) where
  explGet    (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record _ s) <- M.lookup ety <$> readIORef sMap
    return s
  explExists (Space _ sMap _ _ _) ety = liftIO $ M.member ety <$> readIORef sMap

newShape :: SpacePtr -> Ptr Body -> Convex -> Int -> IO (Ptr Shape)
newShape spacePtr' bodyPtr shape (fromIntegral -> ety) = withForeignPtr spacePtr' (go shape)
  where

    go (Convex [toTuple -> (x, y)] (realToFrac -> radius)) spacePtr = [C.block| cpShape* {
      const cpVect vec = { $(float x), $(float y) };
      cpShape* sh = cpCircleShapeNew($(cpBody* bodyPtr), $(float radius), vec);
      cpShapeSetUserData(sh, (void*) $(intptr_t ety));
      return cpSpaceAddShape( $(cpSpace* spacePtr), sh); } |]

    go (Convex [ toTuple -> (xa, ya)
               , toTuple -> (xb, yb) ]
                (realToFrac -> radius)
       ) spacePtr = [C.block| cpShape* {
       const cpVect va = { $(float xa), $(float ya) };
       const cpVect vb = { $(float xb), $(float yb) };
       cpShape* sh = cpSegmentShapeNew($(cpBody* bodyPtr), va, vb, $(float radius));
       cpShapeSetUserData(sh, (void*) $(intptr_t ety));
       return cpSpaceAddShape( $(cpSpace* spacePtr), sh); } |]

    -- go (Convex ((fmap.fmap) realToFrac -> verts)
    -- go (Convex (fmap toTuple -> verts)
    go (Convex verts
               (realToFrac -> radius)
       ) spacePtr = liftIO $ do
         vec <- V.thaw (V.fromList verts)
         [C.block| cpShape* {
           cpTransform trans = cpTransformIdentity;
           cpShape* sh = cpPolyShapeNew($(cpBody* bodyPtr), $vec-len:vec, $vec-ptr:(cpVect *vec), trans, $(float radius));
           cpShapeSetUserData(sh, (void*) $(intptr_t ety));
           return cpSpaceAddShape( $(cpSpace* spacePtr), sh); } |]

destroyShape :: SpacePtr -> Ptr Shape -> IO ()
destroyShape spacePtr shapePtr = withForeignPtr spacePtr $ \space -> [C.block| void {
  cpShape *shape = $(cpShape* shapePtr);
  cpSpaceRemoveShape($(cpSpace* space), shape);
  cpShapeFree (shape); }|]

-- Sensor
getSensor :: Ptr Shape -> IO Bool
getSensor shape = toEnum . fromIntegral <$> [C.exp| int {
  cpShapeGetSensor($(cpShape* shape)) }|]

setSensor :: Ptr Shape -> Bool -> IO ()
setSensor shape (fromIntegral . fromEnum -> isSensor) = [C.exp| void {
  cpShapeSetSensor($(cpShape* shape), $(int isSensor)) }|]

instance Component Sensor where
  type Storage Sensor = Space Sensor
instance (MonadIO m, Has w m Physics) => Has w m Sensor where
  getStore = (cast :: Space Physics -> Space Sensor) <$> getStore

instance MonadIO m => ExplMembers m (Space Sensor) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space Sensor) where
  explSet (Space _ sMap _ _ _) ety (Sensor isSensor) = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setSensor s isSensor

instance MonadIO m => ExplGet m (Space Sensor) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    Sensor <$> getSensor s

-- Elasticity
getElasticity :: Ptr Shape -> IO Float
getElasticity shape = realToFrac <$> [C.exp| float {
  cpShapeGetElasticity($(cpShape* shape)) }|]

setElasticity :: Ptr Shape -> Float -> IO ()
setElasticity shape (realToFrac -> elasticity) = do
  -- print (f, elasticity :: C.CFloat)
  -- print shape
  [C.exp| void { cpShapeSetElasticity($(cpShape* shape), $(float elasticity)) }|]
  -- with elasticity $ \elasticityPtr -> do
  --       -- Print the Haskell pointer and value
  --       ptrValue <- peek elasticityPtr
  --       print $ elasticityPtr
  --       -- printf "elasticity in Haskell (raw pointer): %p\n" (castPtr elasticityPtr :: Ptr ())
  --       -- printf "elasticity in Haskell (value): %f\n" (realToFrac ptrValue :: Float)
  --       -- Call the C function
  --       [C.exp| void { cpShapeSetElasticity($(cpShape* shape), $(float *elasticityPtr)) }|]

instance Component Elasticity where
  type Storage Elasticity = Space Elasticity
instance (MonadIO m, Has w m Physics) => Has w m Elasticity where
  getStore = (cast :: Space Physics -> Space Elasticity) <$> getStore

instance MonadIO m => ExplMembers m (Space Elasticity) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space Elasticity) where
  explSet (Space _ sMap _ _ _) ety (Elasticity elasticity) = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setElasticity s elasticity

instance MonadIO m => ExplGet m (Space Elasticity) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    Elasticity <$> getElasticity s

-- Mass
getMass :: Ptr Shape -> IO Float
getMass shape = realToFrac <$> [C.exp| float {
  cpShapeGetMass($(cpShape* shape)) }|]

setMass :: Ptr Shape -> Float -> IO ()
setMass shape (realToFrac -> mass) = [C.exp| void {
  cpShapeSetMass($(cpShape* shape), $(float mass)) }|]

instance Component Mass where
  type Storage Mass = Space Mass
instance (MonadIO m, Has w m Physics) => Has w m Mass where
  getStore = (cast :: Space Physics -> Space Mass) <$> getStore

instance MonadIO m => ExplMembers m (Space Mass) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space Mass) where
  explSet (Space _ sMap _ _ _) ety (Mass mass) = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setMass s mass

instance MonadIO m => ExplGet m (Space Mass) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    Mass <$> getMass s

-- Density
getDensity :: Ptr Shape -> IO Float
getDensity shape = realToFrac <$> [C.exp| float {
  cpShapeGetDensity($(cpShape* shape)) }|]

setDensity :: Ptr Shape -> Float -> IO ()
setDensity shape (realToFrac -> density) = [C.exp| void {
  cpShapeSetDensity($(cpShape* shape), $(float density)) }|]

instance Component Density where
  type Storage Density = Space Density
instance (MonadIO m, Has w m Physics) => Has w m Density where
  getStore = (cast :: Space Physics -> Space Density) <$> getStore

instance MonadIO m => ExplMembers m (Space Density) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space Density) where
  explSet (Space _ sMap _ _ _) ety (Density density) = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setDensity s density

instance MonadIO m => ExplGet m (Space Density) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    Density <$> getDensity s

-- Friction
getFriction :: Ptr Shape -> IO Float
getFriction shape = realToFrac <$> [C.exp| float {
  cpShapeGetFriction($(cpShape* shape)) }|]

setFriction :: Ptr Shape -> Float -> IO ()
setFriction shape (realToFrac -> friction) = [C.exp| void {
  cpShapeSetFriction($(cpShape* shape), $(float friction)) }|]

instance Component Friction where
  type Storage Friction = Space Friction
instance (MonadIO m, Has w m Physics) => Has w m Friction where
  getStore = (cast :: Space Physics -> Space Friction) <$> getStore

instance MonadIO m => ExplMembers m (Space Friction) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space Friction) where
  explSet (Space _ sMap _ _ _) ety (Friction friction) = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setFriction s friction

instance MonadIO m => ExplGet m (Space Friction) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    Friction <$> getFriction s

-- SurfaceVelocity
getSurfaceVelocity :: Ptr Shape -> IO Vec
getSurfaceVelocity shape = do
 x <- [C.exp| float { cpShapeGetSurfaceVelocity($(cpShape* shape)).x }|]
 y <- [C.exp| float { cpShapeGetSurfaceVelocity($(cpShape* shape)).y }|]
 return (Vec2 (realToFrac x) (realToFrac y))

setSurfaceVelocity :: Ptr Shape -> Vec -> IO ()
setSurfaceVelocity shape (Vec2 (realToFrac -> x) (realToFrac -> y)) = [C.block| void {
  const cpVect vec = { $(float x), $(float y) };
  cpShapeSetSurfaceVelocity($(cpShape* shape), vec);
  }|]

instance Component SurfaceVelocity where
  type Storage SurfaceVelocity = Space SurfaceVelocity
instance (MonadIO m, Has w m Physics) => Has w m SurfaceVelocity where
  getStore = (cast :: Space Physics -> Space SurfaceVelocity) <$> getStore

instance MonadIO m => ExplMembers m (Space SurfaceVelocity) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space SurfaceVelocity) where
  explSet (Space _ sMap _ _ _) ety (SurfaceVelocity svel) = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setSurfaceVelocity s svel

instance MonadIO m => ExplGet m (Space SurfaceVelocity) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    SurfaceVelocity <$> getSurfaceVelocity s

-- CollisionFilter
getFilter :: Ptr Shape -> IO CollisionFilter
getFilter shape = do
 group <- [C.exp| unsigned int { cpShapeGetFilter($(cpShape* shape)).group }|]
 cats  <- [C.exp| unsigned int { cpShapeGetFilter($(cpShape* shape)).categories }|]
 mask  <- [C.exp| unsigned int { cpShapeGetFilter($(cpShape* shape)).mask }|]
 return$ CollisionFilter group (Bitmask cats) (Bitmask mask)

setFilter :: Ptr Shape -> CollisionFilter -> IO ()
setFilter shape (CollisionFilter group (Bitmask cats) (Bitmask mask)) =
  [C.block| void {
    const cpShapeFilter filter = { $(unsigned int group)
                                 , $(unsigned int cats)
                                 , $(unsigned int mask) };
    cpShapeSetFilter($(cpShape* shape), filter);
  }|]

instance Component CollisionFilter where
  type Storage CollisionFilter = Space CollisionFilter
instance (MonadIO m, Has w m Physics) => Has w m CollisionFilter where
  getStore = (cast :: Space Physics -> Space CollisionFilter) <$> getStore

instance MonadIO m => ExplMembers m (Space CollisionFilter) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space CollisionFilter) where
  explSet (Space _ sMap _ _ _) ety cfilter = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setFilter s cfilter

instance MonadIO m => ExplGet m (Space CollisionFilter) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    getFilter s

-- CollisionType
getCollisionType :: Ptr Shape -> IO C.CUIntPtr
getCollisionType shape = [C.exp| uintptr_t {
  cpShapeGetCollisionType($(cpShape* shape)) }|]

setCollisionType :: Ptr Shape -> C.CUIntPtr -> IO ()
setCollisionType shape ctype = [C.exp| void {
  cpShapeSetCollisionType($(cpShape* shape), $(uintptr_t ctype)) }|]

instance Component CollisionType where
  type Storage CollisionType = Space CollisionType
instance (MonadIO m, Has w m Physics) => Has w m CollisionType where
  getStore = (cast :: Space Physics -> Space CollisionType) <$> getStore

instance MonadIO m => ExplMembers m (Space CollisionType) where
  explMembers s = explMembers (cast s :: Space Shape)

instance MonadIO m => ExplSet m (Space CollisionType) where
  explSet (Space _ sMap _ _ _) ety (CollisionType ctype) = liftIO $ do
    rd <- M.lookup ety <$> readIORef sMap
    forM_ rd$ \(Record s _) -> setCollisionType s ctype

instance MonadIO m => ExplGet m (Space CollisionType) where
  explExists s ety = liftIO $ explExists (cast s :: Space Shape) ety
  explGet (Space _ sMap _ _ _) ety = liftIO $ do
    Just (Record s _) <- M.lookup ety <$> readIORef sMap
    CollisionType <$> getCollisionType s
