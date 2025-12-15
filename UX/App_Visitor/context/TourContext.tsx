// context/TourContext.tsx
import React, { createContext, useContext, useState } from "react";

type TourState = {
  tourId: number | null;
  checkpointId: number | null;
  roboId?: number | null;       // 👈 adicionamos roboId aqui
  visitorName?: string | null;
};

type TourContextValue = {
  tour: TourState | null;
  setTour: (value: TourState) => void;
  clearTour: () => void;
};

const TourContext = createContext<TourContextValue | undefined>(undefined);

export const TourProvider = ({ children }: { children: React.ReactNode }) => {
  const [tour, setTourState] = useState<TourState | null>(null);

  const setTour = (value: TourState) => setTourState(value);
  const clearTour = () => setTourState(null);

  return (
    <TourContext.Provider value={{ tour, setTour, clearTour }}>
      {children}
    </TourContext.Provider>
  );
};

export const useTour = () => {
  const ctx = useContext(TourContext);
  if (!ctx) {
    throw new Error("useTour must be used within a TourProvider");
  }
  return ctx;
};
