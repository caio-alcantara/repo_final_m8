import { useState } from "react";

const UserHeader = () => {
  const [isExpanded, setIsExpanded] = useState(false);

  return (
    <div className="bg-[#3C3255]">
      <div className="max-w-[1400px] mx-auto px-10">
        {/* Title row with filters toggle */}
        <div className="flex items-center justify-between py-3">
          <h1 className="text-xl font-bold text-primary-foreground">
            Gestão de usuários
          </h1>
          

        </div>
      </div>
    </div>
  );
};

export default UserHeader;
